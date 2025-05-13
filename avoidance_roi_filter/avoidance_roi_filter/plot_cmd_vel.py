import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import time

class VelocityPlotter(Node):
    def __init__(self):
        super().__init__('velocity_plotter')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.linear = []
        self.angular = []
        self.timestamps = []
        self.start_time = time.time()

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line1, = self.ax.plot([], [], label='linear.x', color='blue')
        self.line2, = self.ax.plot([], [], label='angular.z', color='red')
        self.ax.legend()
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Velocity (m/s & rad/s)')
        self.ax.set_title('TurtleBot3 Velocity')
    
    def listener_callback(self, msg):
        t = time.time() - self.start_time
        self.timestamps.append(t)
        self.linear.append(msg.linear.x)
        self.angular.append(msg.angular.z)

        self.line1.set_xdata(self.timestamps)
        self.line1.set_ydata(self.linear)
        self.line2.set_xdata(self.timestamps)
        self.line2.set_ydata(self.angular)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()