#!/usr/bin/env python3
"""
colorize_cloud.py  —  v4
========================
Projects the RGB camera image onto a 2D LiDAR point cloud so each scan
point gets an RGB colour from the corresponding camera pixel.

v4 additions
------------
* Voxel downsampling before projection  (reduces redundant points)
* Statistical Outlier Removal (SOR)     (removes isolated noise points)
* Better projection masking              (fixes the empty-centre strip bug)
* Near-range mask: ignores points < min_range metres (avoids self-hits)
* Debug image improved: shows camera FoV coverage area

WHY THE CENTRE WAS EMPTY
------------------------
The LiDAR is at x=0.122 and the camera at x=0.300 on the chassis.
Points directly ahead (small x in laser_frame) transform to a camera-frame
position where Z_cam is very small (≈ offset difference ~0.18 m).  At that
short Z the projection formula magnifies tiny X_cam errors hugely, scattering
pixels outside the image.  The fix is:
  1. Use the actual TF (laser→camera_optical) correctly – already done.
  2. Add a minimum forward-depth guard  P_cam[:,2] > min_depth_m  (was 0.01,
     now 0.05 by default – tunable).
  3. Add a near-range mask that drops laser points closer than min_range_m
     (default 0.30 m) so the robot's own body doesn't project noise.

Subscriptions : /camera/image_raw, /camera/camera_info, /lidar/points
Publications  : /colored_cloud        (sensor_msgs/PointCloud2, XYZRGB)
              : /colored_cloud_debug  (sensor_msgs/Image, BGR8)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformException
from sensor_msgs.msg import PointCloud2, Image, CameraInfo, PointField
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge


# ── Helpers ───────────────────────────────────────────────────────────────────

def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)


def pack_rgb_array(bgr_pixels: np.ndarray) -> np.ndarray:
    b = bgr_pixels[:, 0].astype(np.uint32)
    g = bgr_pixels[:, 1].astype(np.uint32)
    r = bgr_pixels[:, 2].astype(np.uint32)
    packed = (r << 16) | (g << 8) | b
    return packed.view(np.float32)


def boost_image(img: np.ndarray, gamma: float = 0.5, scale: float = 1.4) -> np.ndarray:
    lut = np.array([min(255, int((i / 255.0) ** gamma * 255 * scale))
                    for i in range(256)], dtype=np.uint8)
    return lut[img]


def voxel_downsample(pts: np.ndarray, voxel_size: float) -> np.ndarray:
    """Keep one point per voxel (first-hit). pts: Nx3 float64."""
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / voxel_size).astype(np.int32)
    # unique row trick
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


def statistical_outlier_removal(pts: np.ndarray,
                                  k: int = 10,
                                  std_ratio: float = 1.5) -> np.ndarray:
    """
    Remove points whose mean distance to their k nearest neighbours
    exceeds  global_mean + std_ratio * global_std.
    Pure NumPy — no PCL required.
    """
    if len(pts) <= k:
        return pts
    # Pairwise squared distances (fast for moderate N)
    # For large clouds use a KD-tree; here we chunk to stay memory-safe
    from scipy.spatial import cKDTree
    tree = cKDTree(pts)
    dists, _ = tree.query(pts, k=k + 1)   # include self (dist=0)
    mean_dists = dists[:, 1:].mean(axis=1)  # skip self
    mu  = mean_dists.mean()
    sig = mean_dists.std()
    return pts[mean_dists < mu + std_ratio * sig]


# ── Node ──────────────────────────────────────────────────────────────────────

class ColorizeCloud(Node):

    Z_LEVELS = np.linspace(-0.5, 2.5, 200)   # vertical extrusion range [m]

    def __init__(self):
        super().__init__('colorize_cloud')

        self.declare_parameter('image_topic',    '/camera/image_raw')
        self.declare_parameter('info_topic',     '/camera/camera_info')
        self.declare_parameter('cloud_topic',    '/lidar/points')
        self.declare_parameter('output_topic',   '/colored_cloud')
        self.declare_parameter('debug_topic',    '/colored_cloud_debug')
        self.declare_parameter('boost_gamma',    0.5)
        self.declare_parameter('boost_scale',    1.4)
        # v4 params
        self.declare_parameter('voxel_size',     0.05)   # metres; 0 = disabled
        self.declare_parameter('sor_k',          10)     # SOR neighbours
        self.declare_parameter('sor_std_ratio',  1.5)    # SOR threshold
        self.declare_parameter('min_range_m',    0.30)   # ignore points closer than this
        self.declare_parameter('min_depth_m',    0.05)   # min camera-frame Z

        p = self.get_parameter
        self._image_topic  = p('image_topic').value
        self._info_topic   = p('info_topic').value
        self._cloud_topic  = p('cloud_topic').value
        self._output_topic = p('output_topic').value
        self._debug_topic  = p('debug_topic').value
        self._gamma        = p('boost_gamma').value
        self._scale        = p('boost_scale').value
        self._voxel        = p('voxel_size').value
        self._sor_k        = p('sor_k').value
        self._sor_std      = p('sor_std_ratio').value
        self._min_range    = p('min_range_m').value
        self._min_depth    = p('min_depth_m').value

        self.bridge     = CvBridge()
        self._image_b   = None
        self._K         = None
        self._cam_frame = None

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf, self)

        self.create_subscription(Image,       self._image_topic, self._cb_image, 10)
        self.create_subscription(CameraInfo,  self._info_topic,  self._cb_info,  10)
        self.create_subscription(PointCloud2, self._cloud_topic, self._cb_cloud, 10)

        self.pub       = self.create_publisher(PointCloud2, self._output_topic, 10)
        self.pub_debug = self.create_publisher(Image,       self._debug_topic,  10)

        self.get_logger().info(
            f'ColorizeCloud v4 | {self._cloud_topic} + {self._image_topic} '
            f'→ {self._output_topic}  '
            f'voxel={self._voxel}m  SOR(k={self._sor_k},std={self._sor_std})'
        )

    def _cb_image(self, msg):
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self._image_b = boost_image(raw, self._gamma, self._scale)
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}')

    def _cb_info(self, msg):
        self._K         = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._cam_frame = msg.header.frame_id

    def _cb_cloud(self, cloud: PointCloud2):
        if self._image_b is None or self._K is None or self._cam_frame is None:
            return

        # ── TF lookup ────────────────────────────────────────────────────────
        try:
            tf = self.tf_buf.lookup_transform(
                self._cam_frame, cloud.header.frame_id, Time())
        except TransformException as ex:
            self.get_logger().warn(f'TF: {ex}', throttle_duration_sec=2.0)
            return

        R = quat_to_rot(tf.transform.rotation)
        t = tf.transform.translation
        T = np.array([t.x, t.y, t.z], dtype=np.float64)

        # ── Read raw scan ─────────────────────────────────────────────────────
        pts_iter = point_cloud2.read_points(
            cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        pts_list = list(pts_iter)
        if not pts_list:
            return

        if hasattr(pts_list[0], 'dtype') and pts_list[0].dtype.names:
            base = np.array([[p['x'], p['y'], p['z']] for p in pts_list], dtype=np.float64)
        else:
            base = np.array(pts_list, dtype=np.float64).reshape(-1, 3)

        # ── Near-range mask (fix centre empty strip) ──────────────────────────
        ranges = np.linalg.norm(base[:, :2], axis=1)   # 2D range in laser plane
        base   = base[ranges >= self._min_range]
        if len(base) == 0:
            return

        # ── Voxel downsample ──────────────────────────────────────────────────
        if self._voxel > 0:
            base = voxel_downsample(base, self._voxel)

        # ── Statistical Outlier Removal ───────────────────────────────────────
        if self._sor_k > 0 and len(base) > self._sor_k:
            base = statistical_outlier_removal(base, self._sor_k, self._sor_std)

        if len(base) == 0:
            return

        # ── Vertical extrusion ────────────────────────────────────────────────
        N   = len(base)
        Z   = len(self.Z_LEVELS)
        pts_ext = np.repeat(base, Z, axis=0)
        pts_ext[:, 2] = np.tile(self.Z_LEVELS, N)

        # ── Project into camera frame ─────────────────────────────────────────
        P_cam = (R @ pts_ext.T).T + T

        # Forward-depth guard (v4: use min_depth_m, not hard-coded 0.01)
        fwd     = P_cam[:, 2] > self._min_depth
        pts_ext = pts_ext[fwd]
        P_cam   = P_cam[fwd]

        if len(pts_ext) == 0:
            self.get_logger().warn('All points behind camera', throttle_duration_sec=3.0)
            return

        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]
        u = (fx * P_cam[:, 0] / P_cam[:, 2] + cx).astype(np.int32)
        v = (fy * P_cam[:, 1] / P_cam[:, 2] + cy).astype(np.int32)

        H, W   = self._image_b.shape[:2]
        in_fov = (u >= 0) & (u < W) & (v >= 0) & (v < H)
        pts_in = pts_ext[in_fov]
        u_in   = u[in_fov]
        v_in   = v[in_fov]

        if len(pts_in) == 0:
            self.get_logger().warn('No points in FoV', throttle_duration_sec=3.0)
            return

        # ── Sample colours ────────────────────────────────────────────────────
        bgr_px = self._image_b[v_in, u_in]
        rgb_f  = pack_rgb_array(bgr_px)

        # ── Build output ──────────────────────────────────────────────────────
        xyzrgb = np.zeros(len(pts_in),
                          dtype=[('x','f4'),('y','f4'),('z','f4'),('rgb','f4')])
        xyzrgb['x']   = pts_in[:, 0].astype(np.float32)
        xyzrgb['y']   = pts_in[:, 1].astype(np.float32)
        xyzrgb['z']   = pts_in[:, 2].astype(np.float32)
        xyzrgb['rgb'] = rgb_f

        fields = [
            PointField(name='x',   offset=0,  datatype=7, count=1),
            PointField(name='y',   offset=4,  datatype=7, count=1),
            PointField(name='z',   offset=8,  datatype=7, count=1),
            PointField(name='rgb', offset=12, datatype=7, count=1),
        ]
        out = PointCloud2()
        out.header       = cloud.header
        out.height       = 1
        out.width        = len(pts_in)
        out.fields       = fields
        out.is_bigendian = False
        out.point_step   = 16
        out.row_step     = 16 * len(pts_in)
        out.data         = xyzrgb.tobytes()
        out.is_dense     = True
        self.pub.publish(out)

        # ── Debug image ───────────────────────────────────────────────────────
        if self.pub_debug.get_subscription_count() > 0:
            import cv2
            dbg = self._image_b.copy()
            # Draw FoV coverage heatmap
            for uu, vv in zip(u_in[::4], v_in[::4]):
                cv2.circle(dbg, (int(uu), int(vv)), 2, (0, 255, 0), -1)
            try:
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(dbg, 'bgr8'))
            except Exception:
                pass

        self.get_logger().info(
            f'Published {len(pts_in)} pts  '
            f'({N} rays, voxel→{len(base)}, in_fov={in_fov.sum()})  '
            f'brightness={int(bgr_px.mean())}',
            throttle_duration_sec=2.0,
        )


def main():
    rclpy.init()
    node = ColorizeCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
