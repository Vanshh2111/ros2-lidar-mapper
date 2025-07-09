# manual_slicer_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import numpy as np
import math
import struct

BASE_FRAME_ID = "base_link"

class ManualSlicerNode(Node):
    def __init__(self):
        super().__init__('manual_slicer_node')
        self.declare_parameter('tilt_angle_degrees', 0.0)
        self.point_cloud_publisher = self.create_publisher(PointCloud2, 'point_cloud_slice', 10)
        self.laser_scan_subscriber = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.get_logger().info('Manual 3D Slicer Node has started.')
        self.get_logger().info('Set tilt angle with: ros2 param set /manual_slicer_node tilt_angle_degrees <angle>')

    def scan_callback(self, scan_msg):
        points = []
        manual_tilt_angle_deg = self.get_parameter('tilt_angle_degrees').get_parameter_value().double_value
        tilt_angle_rad = math.radians(manual_tilt_angle_deg)
        for i, distance in enumerate(scan_msg.ranges):
            if scan_msg.range_min < distance < scan_msg.range_max:
                lidar_angle_rad = scan_msg.angle_min + i * scan_msg.angle_increment
                x_laser = distance * math.cos(lidar_angle_rad)
                y_laser = distance * math.sin(lidar_angle_rad)
                x_base = x_laser
                y_base = y_laser * math.cos(tilt_angle_rad)
                z_base = y_laser * math.sin(tilt_angle_rad)
                points.append([x_base, y_base, z_base])
        header = scan_msg.header
        header.frame_id = BASE_FRAME_ID
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        packed_data = b"".join([struct.pack('fff', p[0], p[1], p[2]) for p in points])
        point_cloud_msg = PointCloud2(
            header=header, height=1, width=len(points), is_dense=True,
            is_bigendian=False, fields=fields, point_step=12,
            row_step=12 * len(points), data=packed_data
        )
        self.point_cloud_publisher.publish(point_cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ManualSlicerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
