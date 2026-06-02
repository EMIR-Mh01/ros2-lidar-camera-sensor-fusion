#!/usr/bin/env python3
"""
kalman.py
=========
Standalone linear Kalman filter on wheel odometry.

State   : [x, y, vx, vy]   (constant-velocity model)
Input   : /odom             (nav_msgs/Odometry — clean wheel odom)
Outputs : /kalman/odom      (nav_msgs/Odometry — filtered, with covariance)
          /kalman/marker     (visualization_msgs/Marker — RViz sphere)

This node is independent of robot_localization. It demonstrates the raw
Kalman equations. For full sensor fusion with IMU, the EKF node in
local_localization.launch.py is the authoritative filter.

Tuning
------
  Q  — process noise: smaller = smoother (trust the model more)
  R  — measurement noise: larger = filter more aggressively
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import tf2_ros
from geometry_msgs.msg import TransformStamped


class KalmanOdom(Node):

    def __init__(self):
        super().__init__('kalman_filter_odom')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('q_pos', 0.001)   # position process noise
        self.declare_parameter('q_vel', 0.01)    # velocity process noise
        self.declare_parameter('r_pos', 0.05)    # position measurement noise
        self.declare_parameter('r_vel', 0.1)     # velocity measurement noise
        self.declare_parameter('publish_tf', True)

        q_pos = self.get_parameter('q_pos').value
        q_vel = self.get_parameter('q_vel').value
        r_pos = self.get_parameter('r_pos').value
        r_vel = self.get_parameter('r_vel').value
        self._pub_tf = self.get_parameter('publish_tf').value

        # ── Filter matrices ──────────────────────────────────────────────────
        self.dt = 0.1   # updated dynamically from message timestamps

        # State: [x, y, vx, vy]
        self.x = np.zeros((4, 1))
        self.P = np.eye(4) * 1.0

        # State transition (constant-velocity)
        self.A = np.array([
            [1, 0, self.dt, 0],
            [0, 1, 0,       self.dt],
            [0, 0, 1,       0],
            [0, 0, 0,       1],
        ])

        # Measurement: observe [x, y, vx, vy] directly from odom
        self.H = np.eye(4)

        # Process noise covariance Q
        self.Q = np.diag([q_pos, q_pos, q_vel, q_vel])

        # Measurement noise covariance R
        self.R = np.diag([r_pos, r_pos, r_vel, r_vel])

        # ── Time tracking ────────────────────────────────────────────────────
        self._last_time = None

        # ── TF broadcaster ───────────────────────────────────────────────────
        if self._pub_tf:
            self._br = tf2_ros.TransformBroadcaster(self)

        # ── I/O ─────────────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            Odometry, '/odom', self._cb_odom, 10)
        self._pub_odom = self.create_publisher(
            Odometry, '/kalman/odom', 10)
        self._pub_marker = self.create_publisher(
            Marker, '/kalman/marker', 10)

        self.get_logger().info(
            'KalmanOdom ready.\n'
            f'  Q_pos={q_pos}  Q_vel={q_vel}\n'
            f'  R_pos={r_pos}  R_vel={r_vel}\n'
            '  Subscribing to /odom\n'
            '  Publishing  /kalman/odom  and  /kalman/marker')

    # ── callback ──────────────────────────────────────────────────────────────

    def _cb_odom(self, msg: Odometry):
        now = self.get_clock().now()

        # Dynamic dt from message timestamps
        if self._last_time is not None:
            dt = (now - self._last_time).nanoseconds * 1e-9
            if 0.0 < dt < 1.0:
                self.dt = dt
                # Update A with latest dt
                self.A[0, 2] = self.dt
                self.A[1, 3] = self.dt
        self._last_time = now

        # ── Measurement ───────────────────────────────────────────────────────
        z = np.array([
            [msg.pose.pose.position.x],
            [msg.pose.pose.position.y],
            [msg.twist.twist.linear.x],
            [msg.twist.twist.linear.y],
        ])

        # ── Predict ───────────────────────────────────────────────────────────
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

        # ── Update (Kalman gain) ───────────────────────────────────────────────
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        innovation = z - self.H @ self.x
        self.x = self.x + K @ innovation
        self.P = (np.eye(4) - K @ self.H) @ self.P

        # ── Publish filtered Odometry ──────────────────────────────────────────
        out = Odometry()
        out.header.stamp    = now.to_msg()
        out.header.frame_id = 'odom'
        out.child_frame_id  = 'base_footprint_kalman'

        out.pose.pose.position.x = float(self.x[0, 0])
        out.pose.pose.position.y = float(self.x[1, 0])
        out.pose.pose.position.z = 0.0

        # Preserve orientation from raw odom (KF only filters position/vel)
        out.pose.pose.orientation = msg.pose.pose.orientation

        out.twist.twist.linear.x  = float(self.x[2, 0])
        out.twist.twist.linear.y  = float(self.x[3, 0])
        out.twist.twist.angular.z = msg.twist.twist.angular.z

        # Populate covariance from filter's posterior P
        # ROS covariance is 6×6: [x, y, z, rx, ry, rz]
        cov = [0.0] * 36
        cov[0]  = float(self.P[0, 0])   # x-x
        cov[1]  = float(self.P[0, 1])   # x-y
        cov[6]  = float(self.P[1, 0])   # y-x
        cov[7]  = float(self.P[1, 1])   # y-y
        cov[35] = 0.05                   # yaw (not estimated here)
        out.pose.covariance = cov

        vel_cov = [0.0] * 36
        vel_cov[0]  = float(self.P[2, 2])   # vx-vx
        vel_cov[7]  = float(self.P[3, 3])   # vy-vy
        vel_cov[35] = 0.05
        out.twist.covariance = vel_cov

        self._pub_odom.publish(out)

        # ── Publish RViz marker ────────────────────────────────────────────────
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp    = now.to_msg()
        m.ns   = 'kalman'
        m.id   = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r = 1.0
        m.color.g = 0.4
        m.color.b = 0.0
        m.color.a = 0.9
        m.pose.position.x = float(self.x[0, 0])
        m.pose.position.y = float(self.x[1, 0])
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        self._pub_marker.publish(m)

        # ── Optional TF broadcast ──────────────────────────────────────────────
        if self._pub_tf:
            t = TransformStamped()
            t.header.stamp    = now.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id  = 'base_footprint_kalman'
            t.transform.translation.x = float(self.x[0, 0])
            t.transform.translation.y = float(self.x[1, 0])
            t.transform.translation.z = 0.0
            t.transform.rotation = msg.pose.pose.orientation
            self._br.sendTransform(t)


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = KalmanOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
