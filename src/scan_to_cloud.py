#!/usr/bin/env python3
"""
scan_to_cloud.py
================
Converts a 2-D LaserScan into a 3-D PointCloud2 using laser_geometry.
The cloud is published in the laser's own frame so that colorize_cloud.py
can look up the TF to the camera frame at the exact cloud timestamp.

Subscription : /scan          (sensor_msgs/LaserScan)
Publication  : /lidar/points  (sensor_msgs/PointCloud2)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import tf2_ros
from tf2_ros import TransformException


class ScanToCloud(Node):

    def __init__(self):
        super().__init__('scan_to_cloud')

        self.declare_parameter('scan_topic',  '/scan')
        self.declare_parameter('cloud_topic', '/lidar/points')
        self.declare_parameter('channel_options', 0x00)  # add intensity/index channels

        scan_t  = self.get_parameter('scan_topic').value
        cloud_t = self.get_parameter('cloud_topic').value
        channels = self.get_parameter('channel_options').value

        self._lp  = LaserProjection()
        self._ch  = channels

        # TF buffer so laser_geometry can do angular-velocity compensation
        # (not strictly needed for a fixed sensor but keeps things correct)
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lst = tf2_ros.TransformListener(self._tf_buf, self)

        self._sub = self.create_subscription(
            LaserScan, scan_t, self._cb_scan, 10)
        self._pub = self.create_publisher(
            PointCloud2, cloud_t, 10)

        self.get_logger().info(
            f'ScanToCloud: {scan_t} → {cloud_t}')

    def _cb_scan(self, scan: LaserScan):
        try:
            # projectLaser keeps the cloud in the laser frame
            cloud = self._lp.projectLaser(scan, channel_options=self._ch)
        except Exception as e:
            self.get_logger().warn(f'LaserProjection error: {e}')
            return

        # Preserve original frame and stamp
        cloud.header.frame_id = scan.header.frame_id
        cloud.header.stamp    = scan.header.stamp
        self._pub.publish(cloud)


def main():
    rclpy.init()
    node = ScanToCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
