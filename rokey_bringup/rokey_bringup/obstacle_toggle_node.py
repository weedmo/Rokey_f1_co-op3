import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import time
import os

class ObstacleToggleNode(Node):
    def __init__(self):
        super().__init__('obstacle_toggle')
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        self.timer = self.create_timer(5.0, self.toggle_obstacle)
        self.spawned = False

    def toggle_obstacle(self):
        if not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('spawn_entity service not available')
            return
        if not self.delete_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('delete_entity service not available')
            return

        if self.spawned:
            req = DeleteEntity.Request()
            req.name = 'dynamic_obstacle'
            self.delete_cli.call_async(req)
            self.get_logger().info('Obstacle removed')
        else:
            sdf_path = os.path.expanduser('~/team_f_01_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_bringup/worlds/obstacle_box/model.sdf')
            with open(sdf_path, 'r') as f:
                sdf = f.read()

            req = SpawnEntity.Request()
            req.name = 'dynamic_obstacle'
            req.xml = sdf
            req.robot_namespace = ''
            req.reference_frame = 'world'
            req.initial_pose.position.x = -1.24
            req.initial_pose.position.y = -0.5
            req.initial_pose.position.z = 0.05

            self.spawn_cli.call_async(req)
            self.get_logger().info('Obstacle spawned')

        self.spawned = not self.spawned

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleToggleNode()
    rclpy.spin(node)
    rclpy.shutdown()
