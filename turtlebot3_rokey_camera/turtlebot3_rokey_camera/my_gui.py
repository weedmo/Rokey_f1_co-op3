#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from PyQt5.QtWidgets import (
    QApplication, QLabel, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QGridLayout, QFrame, QGraphicsDropShadowEffect
)
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import cv2

MAX_LINEAR_SPEED = 0.15  # m/s 기준값

class ImageWorker(QThread):
    frame_ready = pyqtSignal(QImage)

    def __init__(self, msg):
        super().__init__()
        self.msg = msg

    def run(self):
        # JPEG 디코딩 + RGB 변환
        arr = np.frombuffer(self.msg.data, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # to QImage
        h, w, ch = frame.shape
        qt_img = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888).copy()
        self.frame_ready.emit(qt_img)

class IntegratedGaugeCompassWidget(FigureCanvas):
    def __init__(self):
        fig, ax = plt.subplots()
        super().__init__(fig)
        self.ax = ax
        self.setFixedSize(320, 240)
        self.ax.axis('equal')
        self.ax.axis('off')
        self.update_widget(0.0, 0.0)

    def update_widget(self, linear_x, angular_z, mode_text=""):
        ax = self.ax
        ax.clear()
        ax.axis('equal')
        ax.axis('off')
        # 모드 텍스트
        ax.add_patch(plt.Circle((0, 2), 0.5, fill=False, linewidth=2, edgecolor='white'))
        length, hw, hl = 0.6, 0.3, 0.2
        if abs(angular_z) < 0.05:
            dx, dy = 0, length
        elif angular_z >= 0.25:
            dx, dy = -length, 0
        elif angular_z >= 0.05:
            dx, dy = -length * 0.7, length * 0.7
        elif angular_z <= -0.25:
            dx, dy = length, 0
        elif angular_z <= -0.05:
            dx, dy = length * 0.7, length * 0.7
        else:
            dx, dy = 0, length
        x0, y0 = -dx / 2, -dy / 2 + 2
        ax.arrow(x0, y0, dx, dy, head_width=hw, head_length=hl, fc='black', ec='black', length_includes_head=True)
        # 게이지 arc
        theta = np.linspace(np.pi, 0, 100)
        r = 1.2
        x_outer = r * np.cos(theta)
        y_outer = r * np.sin(theta)
        ax.plot(x_outer, y_outer, color='gray', linewidth=15)
        frac = min(max(linear_x / MAX_LINEAR_SPEED, 0), 1)
        idx = int(frac * len(theta))
        ax.plot(x_outer[:idx], y_outer[:idx], color='blue', linewidth=10)
        # 눈금
        num_ticks, tick_len = 10, 0.1
        for i in range(num_ticks):
            th = np.pi - i * np.pi / (num_ticks - 1)
            x_out, y_out = r * np.cos(th), r * np.sin(th)
            x_in, y_in = (r - tick_len) * np.cos(th), (r - tick_len) * np.sin(th)
            ax.plot([x_in, x_out], [y_in, y_out], color='black', linewidth=2)
        # 속도/각속도 텍스트
        ax.text(0, 0.5, f'{linear_x:.2f} m/s', ha='center', va='top', color='black', fontsize=14)
        ax.text(0, -0.5, f'{angular_z:.2f} rad/s', ha='center', va='top', color='black', fontsize=10)
        self.draw()

class MultiCameraGUI(Node):
    def __init__(self, raw_label, preproc_label, gauge):
        super().__init__('multi_camera_gui')
        self.raw_label = raw_label
        self.preproc_label = preproc_label
        self.gauge = gauge
        self.cmd_vel_pub = None
        self.bridge = CvBridge()
        self._workers = []
        # 구독자
        self.create_subscription(CompressedImage, '/pi_camera/image_raw/compressed', self.raw_callback, 10)
        self.create_subscription(Image, '/pi_camera/preprocessed', self.preproc_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # 모드 라벨
        self.mode_label = QLabel('모드: 자율주행')
        self.mode_label.setAlignment(Qt.AlignCenter)
        self.mode_label.setFixedSize(160, 40)
        self.mode_label.setStyleSheet(
            'color:white; font-weight:bold; font-size:14px;'  
            'background-color:green; border-radius:6px;'
        )

    def raw_callback(self, msg):
        w = ImageWorker(msg)
        w.frame_ready.connect(self.display_raw)
        w.finished.connect(lambda: self._workers.remove(w))
        self._workers.append(w)
        w.start()

    def display_raw(self, img):
        self.raw_label.setPixmap(QPixmap.fromImage(img))
        self.raw_label.setScaledContents(True)

    def preproc_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame.shape
        qt_img = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888).copy()
        self.preproc_label.setPixmap(QPixmap.fromImage(qt_img))
        self.preproc_label.setScaledContents(True)

    def odom_callback(self, msg):
        lin = msg.twist.twist.linear.x
        ang = msg.twist.twist.angular.z
        if self.cmd_vel_pub is None:
            mode, bg = '자율주행', 'green'
        else:
            mode, bg = '수동조작', 'orange'
        self.mode_label.setText(f'모드: {mode}')
        self.mode_label.setStyleSheet(
            f'color:white; font-weight:bold; font-size:14px;'  
            f'background-color:{bg}; border-radius:6px;'
        )
        self.gauge.update_widget(lin, ang, mode_text=mode)

    def send_cmd(self, l, a):
        if self.cmd_vel_pub is None:
            self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        t = Twist()
        t.linear.x = l
        t.angular.z = a
        self.cmd_vel_pub.publish(t)

    def cleanup_threads(self):
        for w in list(self._workers):
            w.quit()
            w.wait()

    def destroy_node(self):
        self.cleanup_threads()
        super().destroy_node()

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    app.setStyleSheet('QWidget { background-color: #2e2e2e; }')

    window = QWidget()
    window.setWindowTitle('TurtleBot3 Multi-Camera GUI')
    main_layout = QVBoxLayout(window)

    # 왼쪽 카메라 두 개
    raw_label = QLabel(); raw_label.setFixedSize(320,240)
    preproc_label = QLabel(); preproc_label.setFixedSize(320,240)

    # 우측: 모드+게이지를 담을 흰색 QFrame
    info_frame = QFrame()
    info_frame.setStyleSheet("""
        background-color: white;
        border: 2px solid lightgray;
        border-radius: 8px;
    """)
    info_layout = QVBoxLayout(info_frame)
    info_layout.setContentsMargins(8, 8, 8, 8)   # 테두리 여유

    gauge = IntegratedGaugeCompassWidget()
    node = MultiCameraGUI(raw_label, preproc_label, gauge)

    # 기존 mode_label 위젯을 그대로 사용
    info_layout.addWidget(node.mode_label, alignment=Qt.AlignCenter)
    info_layout.addWidget(gauge,       alignment=Qt.AlignCenter)

    # 상단 가로 레이아웃: raw, preproc, info_frame
    top_h = QHBoxLayout()
    top_h.addWidget(raw_label)
    top_h.addWidget(preproc_label)
    top_h.addWidget(info_frame) 

    main_layout.addLayout(top_h)
    btn_style = """
        background-color: gray;      /* 배경을 흰색으로 */
        color: white;                 /* 글씨는 검정 */
        border: 1px solid gray;       /* 회색 테두리 */
        border-radius: 4px;           /* 모서리 둥글게 */
        padding: 6px;
    """

    btn_grid = QGridLayout()
    onoff = QPushButton('Auto OFF')
    onoff.setCheckable(True)
    onoff.setStyleSheet(btn_style)

    def toggled(state):
        if state:
            onoff.setText('Auto ON')
            # 항상 흰색 유지
            onoff.setStyleSheet(btn_style)
            node.send_cmd(0.0, 0.0)
        else:
            onoff.setText('Auto OFF')
            # 항상 흰색 유지
            onoff.setStyleSheet(btn_style)
            if node.cmd_vel_pub:
                node.destroy_publisher(node.cmd_vel_pub)
                node.cmd_vel_pub = None

    onoff.clicked.connect(toggled)
    btn_grid.addWidget(onoff, 0, 0)

    def mk_btn(label, l, a):
        b = QPushButton(label)
        b.setStyleSheet(btn_style)  
        b.clicked.connect(lambda: node.send_cmd(l, a))
        return b

    btn_grid.addWidget(mk_btn('Forward', 0.2,  0.0), 0, 1)
    btn_grid.addWidget(mk_btn('Left',    0.0,  0.5), 1, 0)
    btn_grid.addWidget(mk_btn('Right',   0.0, -0.5), 1, 2)
    btn_grid.addWidget(mk_btn('Backward',-0.2, 0.0), 1, 1)
    btn_grid.addWidget(mk_btn('Stop',    0.0,  0.0), 0, 2)

    info_layout.addLayout(btn_grid)
    ...
    # 기존 mode_label 위젯을 그대로 사용
    info_layout.addWidget(node.mode_label, alignment=Qt.AlignCenter)
    info_layout.addWidget(gauge, alignment=Qt.AlignCenter)

    # 슬라이더 및 파라미터 조절 위젯 추가
    from PyQt5.QtWidgets import QSlider, QLabel as QtLabel

    def make_slider(label, min_val, max_val, init_val, step, callback):
        container = QVBoxLayout()
        title = QtLabel(f"{label}: {init_val:.5f}")
        title.setStyleSheet("color: black; font-weight: bold")
        slider = QSlider(Qt.Horizontal)
        slider.setRange(int(min_val / step), int(max_val / step))
        slider.setValue(int(init_val / step))
        slider.valueChanged.connect(lambda val: [
            title.setText(f"{label}: {val * step:.5f}"),
            callback(val * step)
        ])
        container.addWidget(title)
        container.addWidget(slider)
        return container

    def set_gain(val):
        node.set_parameters([rclpy.parameter.Parameter('gain', rclpy.Parameter.Type.DOUBLE, val)])

    def set_max_ang(val):
        node.set_parameters([rclpy.parameter.Parameter('max_ang', rclpy.Parameter.Type.DOUBLE, val)])

    def set_base_lin(val):
        node.set_parameters([rclpy.parameter.Parameter('base_lin', rclpy.Parameter.Type.DOUBLE, val)])
        
    def set_score_threshold(val):
        node.set_parameters([rclpy.parameter.Parameter('score_threshold', rclpy.Parameter.Type.INTEGER, int(val))])

    # 슬라이더들 배치
    sliders_layout = QVBoxLayout()
    sliders_layout.addLayout(make_slider('Gain', 0.000001, 0.00001, 0.00003, 0.000001, set_gain))
    sliders_layout.addLayout(make_slider('Max Angular', 0.1, 0.5, 0.25, 0.01, set_max_ang))
    sliders_layout.addLayout(make_slider('Base Linear', 0.05, 0.3, 0.1, 0.01, set_base_lin))
    sliders_layout.addLayout(make_slider('Score Threshold', 500000, 3000000, 1500000, 10000, set_score_threshold))
    
    info_layout.addLayout(sliders_layout)


    window.setLayout(main_layout)
    window.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    app.exec()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()