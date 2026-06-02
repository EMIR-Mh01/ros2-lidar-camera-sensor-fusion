#!/usr/bin/env python3
"""
ekf_cloud.py  —  EKF Point Cloud Denoiser  v2
==============================================

WHAT IT DOES (simple explanation)
----------------------------------
The raw /colored_cloud is noisy: each LiDAR scan has slightly different
positions for the same wall/object because of sensor jitter.
The EKF remembers where each part of the environment was on the PREVIOUS
scan and blends it with the new measurement → the output cloud is
smoother and more stable over time.

HOW IT WORKS
------------
• The world is divided into a 3D voxel grid (default 10 cm cells).
• Every occupied voxel has its own mini Kalman state: x = [px, py, pz].
• Each cycle:
    PREDICT  – assume the world is static: x_pred = x, P_pred = P + Q
    UPDATE   – new laser measurement z (mean of raw pts in voxel):
               K = P_pred / (P_pred + R)
               x = x_pred + K*(z - x_pred)   ← weighted average
               P = (1-K)*P_pred
• Voxels not seen for `max_age` cycles are deleted (handles robot movement).

KEY FIX vs v1
-------------
v1 stored voxel keys in the SENSOR frame (laser_frame). When the robot
moved, old voxels stayed at the old world position → ghost points everywhere.
v2 works entirely in the MESSAGE FRAME (whatever frame the input cloud uses,
typically `odom` or `map` if you pipe through a static TF, or `laser_frame`
if not). Since the LiDAR cloud is already in `laser_frame` and re-published
each scan, we simply flush any voxel not refreshed in `max_age` ticks —
this naturally handles robot motion without needing odometry.

Subscriptions : /colored_cloud          (XYZRGB PointCloud2)
Publications  : /colored_cloud/filtered (XYZRGB PointCloud2, denoised)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from collections import defaultdict


def unpack_rgb(rgbf: np.ndarray) -> np.ndarray:
    p = rgbf.view(np.uint32)
    r = ((p >> 16) & 0xFF).astype(np.uint8)
    g = ((p >>  8) & 0xFF).astype(np.uint8)
    b = ( p        & 0xFF).astype(np.uint8)
    return np.stack([r, g, b], axis=1)


def pack_rgb(rgb: np.ndarray) -> np.ndarray:
    r = rgb[:, 0].astype(np.uint32)
    g = rgb[:, 1].astype(np.uint32)
    b = rgb[:, 2].astype(np.uint32)
    return ((r << 16) | (g << 8) | b).view(np.float32)


def make_cloud(header, xyz: np.ndarray, rgbf: np.ndarray) -> PointCloud2:
    """Build a valid XYZRGB PointCloud2 (point_step=16, 4-byte aligned)."""
    n = len(xyz)
    buf = np.zeros(n, dtype=[('x','f4'),('y','f4'),('z','f4'),('rgb','f4')])
    buf['x']   = xyz[:, 0]
    buf['y']   = xyz[:, 1]
    buf['z']   = xyz[:, 2]
    buf['rgb'] = rgbf
    msg = PointCloud2()
    msg.header       = header
    msg.height       = 1
    msg.width        = n
    msg.fields       = [
        PointField(name='x',   offset=0,  datatype=7, count=1),  # 7=FLOAT32
        PointField(name='y',   offset=4,  datatype=7, count=1),
        PointField(name='z',   offset=8,  datatype=7, count=1),
        PointField(name='rgb', offset=12, datatype=7, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step   = 16
    msg.row_step     = 16 * n
    msg.data         = buf.tobytes()
    msg.is_dense     = True
    return msg


class EKFCloud(Node):

    def __init__(self):
        super().__init__('ekf_cloud')

        self.declare_parameter('input_topic',   '/colored_cloud')
        self.declare_parameter('output_topic',  '/colored_cloud/filtered')
        self.declare_parameter('voxel_size',    0.10)   # m — grid cell
        self.declare_parameter('q_noise',       0.002)  # process noise
        self.declare_parameter('r_noise',       0.08)   # measurement noise
        self.declare_parameter('max_age',       5)      # flush after N scans
        self.declare_parameter('min_points',    2)      # min pts/voxel to update
        self.declare_parameter('color_alpha',   0.4)    # colour blend speed

        p = self.get_parameter
        in_t         = p('input_topic').value
        out_t        = p('output_topic').value
        self._vs     = p('voxel_size').value
        self._q      = p('q_noise').value
        self._r      = p('r_noise').value
        self._maxage = p('max_age').value
        self._minpts = p('min_points').value
        self._alpha  = p('color_alpha').value

        # state: key → {'x': [3], 'P': float, 'col': [3], 'age': int}
        self._voxels = {}

        self._sub = self.create_subscription(PointCloud2, in_t,  self._cb, 10)
        self._pub = self.create_publisher(PointCloud2,    out_t, 10)

        self.get_logger().info(
            f'EKFCloud v2  {in_t} → {out_t}\n'
            f'  voxel={self._vs}m  Q={self._q}  R={self._r}  max_age={self._maxage}'
        )

    def _cb(self, msg: PointCloud2):
        pts = list(point_cloud2.read_points(
            msg, field_names=('x','y','z','rgb'), skip_nans=True))
        if not pts:
            return

        if hasattr(pts[0], 'dtype') and pts[0].dtype.names:
            xyz  = np.array([[p['x'],p['y'],p['z']] for p in pts], np.float32)
            rgbf = np.array([p['rgb'] for p in pts], np.float32)
        else:
            arr  = np.array(pts, np.float32).reshape(-1, 4)
            xyz, rgbf = arr[:,:3], arr[:,3]

        rgb = unpack_rgb(rgbf)   # Nx3 uint8

        # bucket points into voxels
        keys = [tuple(k) for k in np.floor(xyz / self._vs).astype(np.int32)]
        buckets = defaultdict(list)
        for i, k in enumerate(keys):
            buckets[k].append(i)

        # predict (age all voxels)
        for v in self._voxels.values():
            v['P']   = v['P'] + self._q
            v['age'] += 1

        # update observed voxels
        for k, idxs in buckets.items():
            if len(idxs) < self._minpts:
                continue
            ia = np.array(idxs)
            z  = xyz[ia].mean(0).astype(np.float64)
            c  = rgb[ia].mean(0).astype(np.float64)
            if k in self._voxels:
                v = self._voxels[k]
                # scalar Kalman gain (same for all 3 dims since P is scalar)
                K    = v['P'] / (v['P'] + self._r)
                v['x']   = v['x']   + K * (z - v['x'])
                v['P']   = (1 - K)  * v['P']
                v['col'] = (1 - self._alpha) * v['col'] + self._alpha * c
                v['age'] = 0
            else:
                self._voxels[k] = {'x': z, 'P': self._r, 'col': c, 'age': 0}

        # prune stale voxels
        self._voxels = {k: v for k, v in self._voxels.items()
                        if v['age'] <= self._maxage}

        if not self._voxels:
            return

        vals = list(self._voxels.values())
        out_xyz = np.array([v['x'] for v in vals], np.float32)
        out_col = np.clip([v['col'] for v in vals], 0, 255).astype(np.uint8)
        out_f   = pack_rgb(out_col)

        self._pub.publish(make_cloud(msg.header, out_xyz, out_f))
        self.get_logger().info(
            f'EKF: {len(vals)} voxels (in={len(xyz)})',
            throttle_duration_sec=2.0)


def main():
    rclpy.init()
    n = EKFCloud()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
