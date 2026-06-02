# my_robot_description — Perception Pipeline


### Bug fix: Empty centre strip in coloured cloud
**Root cause:** The LiDAR is at `x=0.122 m` and the camera at `x=0.300 m` on
the chassis. Points directly ahead of the robot are very close in range, which
means after the `laser_frame → camera_optical` TF transform their camera-frame
Z depth (`P_cam_z`) is tiny (~0.18 m). The pinhole projection formula
`u = fx * Px/Pz` amplifies small X errors when Pz is small, scattering pixels
far outside the image.

**Fixes applied in `colorize_cloud.py` v4:**
- `min_range_m = 0.30` — drops LiDAR rays closer than 30 cm (robot self-hits)
- `min_depth_m = 0.05` — tighter camera-frame forward-depth guard
- Voxel downsample before projection reduces redundant rays
- Statistical Outlier Removal (SOR) removes isolated noise points

---

## Node overview

| Node | Input | Output | Purpose |
|------|-------|--------|---------|
| `scan_to_cloud.py` | `/scan` | `/lidar/points` | LaserScan → PointCloud2 |
| `colorize_cloud.py` v4 | `/lidar/points` + `/camera/image_raw` | `/colored_cloud` | RGB projection + voxel filter + SOR |
| `depth_colorize_cloud.py` | `/depth_camera/points` + image | `/depth_camera/colored_points` | Depth camera colourisation |
| `ekf_cloud.py` | `/colored_cloud` | `/colored_cloud/filtered` | EKF spatial denoising |
| `segment_cloud.py` | `/colored_cloud/filtered` | `/segmented_cloud` + `/segment_markers` | Semantic segmentation |

---

## Segmentation colour legend

| Class    | Colour  | Criteria |
|----------|---------|----------|
| Ground   | Grey    | Z ∈ [-0.20, 0.08] m |
| Wall     | Blue    | Height > 1.2 m, thin in one horizontal axis |
| Person   | Yellow  | Footprint < 0.6 m², height 0.8–2.2 m |
| Table    | Orange  | Top height 0.4–1.1 m, thin vertically |
| Car      | Red     | Footprint > 1.5 m², height 0.4–2.5 m |
| Obstacle | Cyan    | Everything else |

---

## How to build and run

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash

# Terminal 1 — Gazebo + robot
ros2 launch my_robot_description robot_gazebo.launch.py

# Terminal 2 — Localisation (EKF odom)
ros2 launch my_robot_description local_localization.launch.py

# Terminal 3 — Full perception pipeline
ros2 launch my_robot_description perception_pipeline.launch.py

# Terminal 4 — RViz
ros2 launch my_robot_description display.launch.py
```

---

## RViz topics to enable

| Topic | Type | Notes |
|-------|------|-------|
| `/colored_cloud` | PointCloud2 | Raw colourised LiDAR |
| `/colored_cloud/filtered` | PointCloud2 | EKF-denoised |
| `/segmented_cloud` | PointCloud2 | Semantic classes |
| `/segment_markers` | MarkerArray | Bounding boxes + labels |
| `/depth_camera/colored_points` | PointCloud2 | Depth camera |
| `/colored_cloud_debug` | Image | Projection coverage debug |

---

## Tuning guide

### Still seeing gaps in the coloured cloud?
- Decrease `min_range_m` toward `0.15` if the gap persists at short range
- Increase `min_range_m` toward `0.50` if robot body artifacts appear

### Too much noise?
- Decrease `sor_std_ratio` to `1.0` (more aggressive SOR)
- Increase EKF `r_noise` to `0.10` (trust measurements less)

### Wrong segmentation classes?
- The classifier uses pure geometry heuristics — no ML required
- Adjust thresholds in `segment_cloud.py → classify_cluster()`
- Increase `cluster_tolerance` (default 0.25 m) if objects are split in two
- Decrease it if different objects merge into one cluster

### EKF latency / ghosting?
- Reduce `max_age` from 15 to 5 for a more reactive (less smooth) map
- Reduce `q_noise` toward 0.0001 for maximum smoothing of a truly static scene
