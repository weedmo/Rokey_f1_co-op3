import struct
import math
import sys
import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField

class PCDListener(Node):
    # ROS2 노드에서 포인트 클라우드 데이터를 수신하고 Open3D로 시각화
    def __init__(self):
        super().__init__('pcd_subscriber_node')
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.pcd_subscriber = self.create_subscription(
            sensor_msgs.PointCloud2,
            'pcd',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # 포인트 클라우드 데이터를 처리하고 시각화
        points = read_points(msg, field_names=("x", "y", "z"))
        pcd_as_numpy_array = np.array([point[:3] for point in points])

        if pcd_as_numpy_array.shape[0] == 0:
            self.get_logger().error("Received empty point cloud")
            return

        self.vis.remove_geometry(self.o3d_pcd)
        self.o3d_pcd = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(pcd_as_numpy_array)
        )
        self.vis.add_geometry(self.o3d_pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

_DATATYPES = {
    PointField.INT8: ('b', 1),
    PointField.UINT8: ('B', 1),
    PointField.INT16: ('h', 2),
    PointField.UINT16: ('H', 2),
    PointField.INT32: ('i', 4),
    PointField.UINT32: ('I', 4),
    PointField.FLOAT32: ('f', 4),
    PointField.FLOAT64: ('d', 8)
}

def read_points(cloud, field_names=None):
    # PointCloud2 메시지에서 포인트 데이터를 추출
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)

    width = cloud.width
    height = cloud.height
    point_step = cloud.point_step
    row_step = cloud.row_step
    data = cloud.data

    unpack_from = struct.Struct(fmt).unpack_from

    for v in range(height):
        offset = row_step * v
        for u in range(width):
            yield unpack_from(data, offset)
            offset += point_step

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    # PointCloud2 필드 정보를 바탕으로 구조체 포맷 문자열 생성
    fmt = '>' if is_bigendian else '<'
    offset = 0
    fields_sorted = sorted(fields, key=lambda f: f.offset)
    for field in (f for f in fields_sorted if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype in _DATATYPES:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length
        else:
            print(f'Skipping unknown PointField datatype [{field.datatype}]', file=sys.stderr)
    return fmt

def main(args=None):
    # ROS2 노드를 초기화하고 실행
    rclpy.init(args=args)
    pcd_listener = PCDListener()
    rclpy.spin(pcd_listener)
    pcd_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

