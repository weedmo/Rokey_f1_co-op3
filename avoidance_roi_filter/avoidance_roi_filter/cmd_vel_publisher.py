import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher_node')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_input', 10)
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('cmd_vel_publisher_node START!')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.3   
        msg.angular.z = 0.0  
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()