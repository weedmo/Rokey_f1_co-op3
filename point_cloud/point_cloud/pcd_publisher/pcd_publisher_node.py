import sys
import os
import struct
import rclpy
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import numpy as np
import open3d as o3d
from rclpy.node import Node


class PCDPublisher(Node):
    #PCD 데이터를 퍼블리시하는 ROS2 노드
    def __init__(self, voxel_size, pcd_path):
        super().__init__('pcd_publisher_node')
        self.voxel_size = voxel_size
        self.pcd_path = pcd_path

        self.load_point_cloud()
        self.points = self.rotate_points_90(self.points)
        self.points[:, 2] += 2.5

        self.pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2, 'pcd', 10)
        self.timer = self.create_timer(1 / 30.0, self.timer_callback)

    def load_point_cloud(self):
        #포인트 클라우드 파일을 로드
        if not os.path.exists(self.pcd_path):
            raise FileNotFoundError("File doesn't exist.")

        pcd = o3d.io.read_point_cloud(self.pcd_path)
        if self.voxel_size > 0:
            pcd = pcd.voxel_down_sample(self.voxel_size)

        self.points = np.asarray(pcd.points)
        self.colors = np.asarray(pcd.colors)

    def timer_callback(self):
        #타이머 콜백 : points와 colors 데이터를 이용해 PointCloud 메시지를 생성
        #좌표계 : map을 기준으로 함
        pcd_msg = self.create_point_cloud_message(self.points, self.colors, 'map')
        self.pcd_publisher.publish(pcd_msg)

    def rotate_points_90(self, points):
        #포인트 클라우드를 X축 기준으로 90도 회전
        theta = np.radians(90)
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ])
        return np.dot(points, rotation_matrix)

    def create_point_cloud_message(self, points, colors, parent_frame):
        #포인트 클라우드 메시지를 생성
        ros_dtype = sensor_msgs.PointField.FLOAT32
        data = [
            struct.pack('fffI', x, y, z, (int(r * 255) << 16) | (int(g * 255) << 8) | int(b * 255))
            for (x, y, z), (r, g, b) in zip(points, colors)
        ]
        data = b''.join(data)

        fields = [
            sensor_msgs.PointField(name='x', offset=0, datatype=ros_dtype, count=1),
            sensor_msgs.PointField(name='y', offset=4, datatype=ros_dtype, count=1),
            sensor_msgs.PointField(name='z', offset=8, datatype=ros_dtype, count=1),
            sensor_msgs.PointField(name='rgb', offset=12, datatype=sensor_msgs.PointField.UINT32, count=1),
        ]

        header = std_msgs.Header(frame_id=parent_frame)
        return sensor_msgs.PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=16,
            row_step=16 * points.shape[0],
            data=data
        )

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) <= 1:
        raise ValueError("ply 파일이 필요합니다.")

    pcd_path = sys.argv[1]
    voxel_size = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

    pcd_publisher = PCDPublisher(voxel_size, pcd_path)
    rclpy.spin(pcd_publisher)
    pcd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

