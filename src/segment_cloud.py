#!/usr/bin/env python3
"""
segment_cloud.py  —  v2
========================

WHY THE PREVIOUS VERSION DIDN'T SEPARATE OBJECTS
-------------------------------------------------
The previous version tried to cluster the FULL vertically-extruded LiDAR
curtain (each ray → 200 Z-levels → one tall column of points).  Two
adjacent columns from different objects were only 0.25 m apart in XY, so
the Euclidean clusterer joined them all into ONE giant cluster.

THE FIX — cluster in 2D (XY) on the raw laser scan ring, then extrude
-------------------------------------------------------------------
Step 1:  Read the EKF-filtered cloud but PROJECT every point to Z=0
         (i.e. use only the XY position).  This collapses the curtain back
         to a flat ring of 2D points — exactly like the original laser scan.

Step 2:  Run Euclidean 2D clustering on those projected points.
         At this level objects ARE separated because adjacent walls/people
         have a clear gap in XY.

Step 3:  For each 2D cluster collect ALL the 3D points (all Z levels) that
         belong to the same XY origin ray.  This gives each object its full
         3D vertical extent.

Step 4:  Classify by bounding-box geometry (wall / person / table / car /
         obstacle) and assign a colour.

SEGMENTATION COLOUR LEGEND
--------------------------
  Grey    [128,128,128] — Ground   (Z band near floor)
  Blue    [ 50,100,255] — Wall     (tall + thin)
  Yellow  [255,220,  0] — Person   (narrow footprint, 1–2 m tall)
  Orange  [255,140,  0] — Table    (flat top 0.5–1.1 m)
  Red     [220, 40, 40] — Car      (large footprint)
  Cyan    [  0,220,200] — Obstacle (everything else)

Subscriptions : /colored_cloud/filtered
Publications  : /segmented_cloud  ,  /segment_markers
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from scipy.spatial import cKDTree


# ── Classes & colours ─────────────────────────────────────────────────────────
GROUND, WALL, PERSON, TABLE, CAR, OBSTACLE = 0, 1, 2, 3, 4, 5
NAMES  = ['Ground', 'Wall', 'Person', 'Table', 'Car', 'Obstacle']
COLORS = np.array([
    [128, 128, 128],
    [ 50, 100, 255],
    [255, 220,   0],
    [255, 140,   0],
    [220,  40,  40],
    [  0, 220, 200],
], dtype=np.uint8)


def pack_rgb(rgb: np.ndarray) -> np.ndarray:
    r = rgb[:,0].astype(np.uint32)
    g = rgb[:,1].astype(np.uint32)
    b = rgb[:,2].astype(np.uint32)
    return ((r<<16)|(g<<8)|b).view(np.float32)


def make_cloud(header, xyz, rgbf) -> PointCloud2:
    n   = len(xyz)
    buf = np.zeros(n, dtype=[('x','f4'),('y','f4'),('z','f4'),('rgb','f4')])
    buf['x'], buf['y'], buf['z'], buf['rgb'] = xyz[:,0], xyz[:,1], xyz[:,2], rgbf
    msg = PointCloud2()
    msg.header       = header
    msg.height, msg.width = 1, n
    msg.fields = [
        PointField(name='x',   offset=0,  datatype=7, count=1),
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


# ── 2-D Euclidean clustering ──────────────────────────────────────────────────

def cluster_2d(xy: np.ndarray, tol: float,
               min_sz: int, max_sz: int) -> list:
    """BFS clustering on XY plane. Returns list of index arrays."""
    if len(xy) == 0:
        return []
    tree    = cKDTree(xy)
    visited = np.zeros(len(xy), bool)
    out     = []
    for seed in range(len(xy)):
        if visited[seed]:
            continue
        q, members = [seed], []
        while q:
            i = q.pop()
            if visited[i]:
                continue
            visited[i] = True
            members.append(i)
            q.extend(nb for nb in tree.query_ball_point(xy[i], tol)
                     if not visited[nb])
        if min_sz <= len(members) <= max_sz:
            out.append(np.array(members, np.int32))
    return out


# ── Geometry classifier ───────────────────────────────────────────────────────

def classify(pts3d: np.ndarray) -> int:
    mn = pts3d.min(0); mx = pts3d.max(0)
    dx, dy, dz = float(mx[0]-mn[0]), float(mx[1]-mn[1]), float(mx[2]-mn[2])
    z_top  = float(mx[2])
    fp     = max(dx * dy, 0.0)

    # PERSON first: standing human is tall but has a very small footprint.
    if dz >= 1.0 and dz <= 2.4 and fp < 1.2 and max(dx, dy) < 1.2:
        return PERSON

    # WALL: long planar structure.
    if dz > 1.5 and (dx > 2.0 or dy > 2.0):
        return WALL

    if 0.2 <= fp <= 4.0 and 0.4 <= z_top <= 1.15 and dz < 0.7:
        return TABLE

    # Vehicle-sized objects
    if (fp > 1.5 and dz > 0.8) or (max(dx, dy) > 1.5 and dz > 0.8):
        return CAR

    return OBSTACLE


# ── Node ──────────────────────────────────────────────────────────────────────

class SegmentCloud(Node):

    def __init__(self):
        super().__init__('segment_cloud')

        self.declare_parameter('input_topic',        '/colored_cloud/filtered')
        self.declare_parameter('output_topic',       '/segmented_cloud')
        self.declare_parameter('marker_topic',       '/segment_markers')
        # 2-D clustering params (applied to the XY ring)
        self.declare_parameter('cluster_tolerance',  0.08)   # m in XY
        self.declare_parameter('min_cluster_size',   5)
        self.declare_parameter('max_cluster_size',   2000)
        # Ground band
        self.declare_parameter('ground_z_min',      -0.25)
        self.declare_parameter('ground_z_max',       0.10)
        self.declare_parameter('remove_ground',      True)

        p = self.get_parameter
        in_t        = p('input_topic').value
        out_t       = p('output_topic').value
        mrk_t       = p('marker_topic').value
        self._tol   = p('cluster_tolerance').value
        self._min   = p('min_cluster_size').value
        self._max   = p('max_cluster_size').value
        self._gz_lo = p('ground_z_min').value
        self._gz_hi = p('ground_z_max').value
        self._rmgnd = p('remove_ground').value

        self._sub    = self.create_subscription(PointCloud2, in_t,  self._cb, 10)
        self._pub    = self.create_publisher(PointCloud2,    out_t, 10)
        self._pub_mk = self.create_publisher(MarkerArray,    mrk_t, 10)

        self.get_logger().info(
            f'SegmentCloud v2  {in_t} → {out_t}\n'
            f'  2D cluster tol={self._tol}m  min={self._min}  max={self._max}\n'
            f'  Ground: Z ∈ [{self._gz_lo}, {self._gz_hi}] m'
        )

    def _cb(self, msg: PointCloud2):
        pts = list(point_cloud2.read_points(
            msg, field_names=('x','y','z','rgb'), skip_nans=True))
        if len(pts) < self._min:
            return

        if hasattr(pts[0], 'dtype') and pts[0].dtype.names:
            xyz = np.array([[p['x'],p['y'],p['z']] for p in pts], np.float32)
        else:
            xyz = np.array(pts, np.float32).reshape(-1,4)[:,:3]

        # ── Separate ground ───────────────────────────────────────────────────
        gnd_mask = (xyz[:,2] >= self._gz_lo) & (xyz[:,2] <= self._gz_hi)
        xyz_gnd  = xyz[gnd_mask]
        xyz_obj  = xyz[~gnd_mask] if self._rmgnd else xyz

        if len(xyz_obj) == 0:
            return

        # ── STEP 1: collapse to 2D XY  ────────────────────────────────────────
        # Round XY to a coarse grid so nearby curtain columns get the same key.
        # This groups the ~200 Z-extruded copies of each laser ray back into
        # one 2D point before clustering.
        GRID = 0.05   # 5 cm — match voxel size used in EKF/colorize
        xy_keys  = np.round(xyz_obj[:, :2] / GRID).astype(np.int32)
        # unique XY positions (index of first occurrence)
        _, ring_idx = np.unique(xy_keys, axis=0, return_index=True)
        ring_idx    = np.sort(ring_idx)
        xy_ring     = xyz_obj[ring_idx, :2].astype(np.float64)   # 2D ring

        # ── STEP 2: 2D Euclidean clustering on XY ring ────────────────────────
        clusters_2d = cluster_2d(xy_ring, self._tol, self._min, self._max)

        if not clusters_2d:
            self.get_logger().info('No clusters', throttle_duration_sec=2.0)
            return

        # ── STEP 3: map 2D clusters back to 3D points ─────────────────────────
        # For each 2D cluster member (which is an index into ring_idx),
        # find all 3D points that share the same rounded XY key.
        out_xyz  = []
        out_rgbf = []
        markers  = []
        mcount   = 0

        # Build a lookup: xy_key → list of indices in xyz_obj
        from collections import defaultdict
        key_to_3d = defaultdict(list)
        for i in range(len(xyz_obj)):
            key_to_3d[tuple(xy_keys[i])].append(i)

        # Ground → grey
        if self._rmgnd and len(xyz_gnd) > 0:
            gc = np.tile(COLORS[GROUND], (len(xyz_gnd), 1))
            out_xyz.append(xyz_gnd)
            out_rgbf.append(pack_rgb(gc))

        counts = {n: 0 for n in NAMES}

        for clust_idx in clusters_2d:
            # collect all 3D points from the 2D cluster members
            pts3d_idx = []
            for ring_i in clust_idx:
                orig_i   = ring_idx[ring_i]
                key      = tuple(xy_keys[orig_i])
                pts3d_idx.extend(key_to_3d[key])

            if not pts3d_idx:
                continue
            pts3d = xyz_obj[np.array(pts3d_idx, np.int32)]

            cls   = classify(pts3d)
            col   = np.tile(COLORS[cls], (len(pts3d), 1))
            out_xyz.append(pts3d)
            out_rgbf.append(pack_rgb(col))
            counts[NAMES[cls]] += 1

            # bounding box marker
            mn = pts3d.min(0); mx = pts3d.max(0)
            ctr = ((mn+mx)/2).astype(float)
            dim = (mx-mn).astype(float)
            c   = COLORS[cls].astype(float)/255.

            bx = Marker()
            bx.header = msg.header
            bx.ns, bx.id   = 'seg', mcount;  mcount += 1
            bx.type        = Marker.CUBE
            bx.action      = Marker.ADD
            bx.pose.position.x = float(ctr[0])
            bx.pose.position.y = float(ctr[1])
            bx.pose.position.z = float(ctr[2])
            bx.pose.orientation.w = 1.0
            bx.scale.x = max(float(dim[0]), 0.05)
            bx.scale.y = max(float(dim[1]), 0.05)
            bx.scale.z = max(float(dim[2]), 0.05)
            bx.color   = ColorRGBA(r=c[0], g=c[1], b=c[2], a=0.30)
            bx.lifetime.sec = 1
            markers.append(bx)

            lb = Marker()
            lb.header = msg.header
            lb.ns, lb.id   = 'lbl', mcount;  mcount += 1
            lb.type        = Marker.TEXT_VIEW_FACING
            lb.action      = Marker.ADD
            lb.pose.position.x = float(ctr[0])
            lb.pose.position.y = float(ctr[1])
            lb.pose.position.z = float(mx[2]) + 0.20
            lb.pose.orientation.w = 1.0
            lb.scale.z     = 0.18
            lb.color       = ColorRGBA(r=1., g=1., b=1., a=1.)
            lb.text        = f'{NAMES[cls]}\n{len(pts3d)}pts'
            lb.lifetime.sec = 1
            markers.append(lb)

        if not out_xyz:
            return

        all_xyz  = np.vstack(out_xyz).astype(np.float32)
        all_rgbf = np.concatenate(out_rgbf).astype(np.float32)

        self._pub.publish(make_cloud(msg.header, all_xyz, all_rgbf))
        self._pub_mk.publish(MarkerArray(markers=markers))
        self.get_logger().info(
            f'Segments: {len(clusters_2d)} clusters  {counts}',
            throttle_duration_sec=2.0)


def main():
    rclpy.init()
    n = SegmentCloud()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
