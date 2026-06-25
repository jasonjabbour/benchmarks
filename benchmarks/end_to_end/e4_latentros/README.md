# e4_latentros — LatentROS Grey-Box Benchmark Suite

Grey-box workload characterization across the **full autonomous quadruped (Unitree GO2)
navigation stack**. Measures `T_in`, `T_kernel`, `T_out` for 11 kernels spanning
Perception, Mapping, Localization, Planning, and Control using nanosecond-precision
LTTng tracepoints.

---

## Architecture

Each benchmark follows the **intercept pattern**:

```
rosbag (looped) → [Input Component] → /robotperf/benchmark/<topic>
    → [Kernel Node (with tracepoints)] → [Output Component]
```

Every kernel has been forked to add two tracepoints inside:
- `robotperf_msg_received_1` — fires when the kernel receives input
- `robotperf_msg_published_1` — fires just before the kernel publishes output

A **unique key** is propagated via `header.stamp.nanosec` from input → kernel → output
so each message can be correlated across all 6 tracepoints.

### 6-Tracepoint Chain (per message)

```
1. <type>_input_cb_init      — Input component receives message from rosbag
2. <type>_input_cb_fini      — Input component publishes to benchmark namespace
3. robotperf_msg_received_1  — Kernel receives the message
4. robotperf_msg_published_1 — Kernel publishes output
5. <type>_output_cb_init     — Output component receives kernel output
6. <type>_output_cb_fini     — Output component finishes
```

This gives 5 timing intervals:
| Interval | Meaning |
|---|---|
| `init → fini` (input) | T_in: input component overhead |
| `input_fini → msg_received` | DDS transport: input → kernel |
| `msg_received → msg_published` | **T_kernel**: pure computation |
| `msg_published → output_init` | DDS transport: kernel → output |
| `init → fini` (output) | T_out: output component overhead |

---

## Benchmark Table

Ordered by the robotics pipeline: Perception → Mapping → Localization → Planning → Control.

### Perception (8 benchmarks)

| # | Algorithm | Kernel Package | Input | Output | Session | Description |
|---|---|---|---|---|---|---|
| 1 | PointCloud→LaserScan | `pointcloud_to_laserscan` | PointCloud2 | LaserScan | `e4_pc_to_laserscan` | Projects 3D point cloud into 2D laser scan by computing range and angle for each point and binning into angular sectors. |
| 2 | Image Resize | `image_proc` | Image | Image | `e4_image_resize` | Scales a camera image by a configurable factor (default 0.5x) using OpenCV interpolation. Common preprocessing step for reducing compute on downstream nodes. |
| 3 | DepthImage→LaserScan | `depthimage_to_laserscan` | DepthImage | LaserScan | `e4_depthimage_to_laserscan` | Extracts a horizontal scanline from a depth camera image and converts it into a 2D laser scan. Enables use of depth cameras with algorithms that expect LaserScan input. |
| 4 | Stereo Disparity | `stereo_image_proc` | 2x Image | DisparityImage | `e4_stereo_disparity` | Computes a disparity map from a stereo camera pair (left + right images) using block matching. Disparity encodes depth information for each pixel. |
| 5 | DepthImage→PointCloud | `depth_image_proc` | DepthImage+RGB | PointCloud2 | `e4_depthimage_to_pointcloud` | Reprojects a depth image into a 3D point cloud and fuses it with a color image to produce an XYZRGB point cloud. Core step in RGB-D perception pipelines. |
| 6 | LaserScan Range Filter | `laser_filters` | LaserScan | LaserScan | `e4_laserscan_range_filter` | Filters laser scan readings to keep only points within a configurable distance range (0.3–10.0m), removing noise and out-of-range readings. |
| 7 | LaserScan Median Filter | `laser_filters` | LaserScan | LaserScan | `e4_laserscan_median_filter` | Applies a sliding-window median filter to smooth laser scan readings, reducing salt-and-pepper noise while preserving edges. |

### Mapping (3 benchmarks)

| # | Algorithm | Kernel Package | Input | Output | Session | Description |
|---|---|---|---|---|---|---|
| 8 | Nav2 Global Costmap | `nav2_costmap_2d` | LaserScan | OccupancyGrid | `e4_global_costmap` | Builds a full-map obstacle grid by fusing the static map with live laser scan detections. Used by the global planner to find collision-free paths. |
| 9 | Nav2 Local Costmap | `nav2_costmap_2d` | LaserScan | OccupancyGrid | `e4_local_costmap` | Maintains a small rolling-window obstacle grid around the robot using a 3D voxel layer. Used by the local controller for reactive obstacle avoidance. |
| 10 | SLAM Toolbox | `slam_toolbox` | LaserScan | OccupancyGrid | `e4_slam_toolbox` | Online graph-based SLAM — incrementally builds an occupancy grid map from laser scans while simultaneously localizing the robot. |

### Localization (4 benchmarks)

| # | Algorithm | Kernel Package | Input | Output | Session | Description |
|---|---|---|---|---|---|---|
| 11 | AMCL (Particle Filter) | `nav2_amcl` | LaserScan | PoseWithCov | `e4_amcl` | Estimates the robot's pose on the map using Adaptive Monte Carlo Localization — a particle filter that matches laser scans against the known map. |
| 12 | EKF base→footprint | `robot_localization` | PoseWithCov | Odometry | `e4_base_to_footprint_ekf` | Fuses body pose estimates from leg kinematics with IMU data using an Extended Kalman Filter to produce a smooth body orientation estimate. |
| 13 | EKF footprint→odom | `robot_localization` | Odometry | Odometry | `e4_footprint_to_odom_ekf` | Fuses raw wheel/leg odometry with IMU yaw data using an EKF to produce drift-corrected global position tracking. |
| 14 | CHAMP State Estimation | `champ_base` | JointState | Odometry | `e4_state_estimation` | Computes raw odometry from 12 leg joint angles using forward kinematics — determines how fast and in what direction the quadruped is moving. |

### Planning (5 benchmarks)

| # | Algorithm | Kernel Package | Input | Output | Session | Description |
|---|---|---|---|---|---|---|
| 15 | NavFn A* Global Planner | `navfn_benchmark_node` | PoseStamped | Path | `e4_global_planner` | Computes the shortest collision-free path on the costmap using the A* search algorithm. Topic-driven wrapper for benchmarking. |
| 16 | Theta* Planner | `theta_star_benchmark_node` | PoseStamped | Path | `e4_theta_star_planner` | Any-angle path planning — finds shorter paths than A* by allowing non-grid-aligned moves between cells. Topic-driven wrapper. |
| 17 | OMPL RRT-Connect | `rrt_benchmark_node` | PoseStamped | Path | `e4_rrt_planner` | Sampling-based path planning using OMPL's RRT-Connect algorithm. Fundamentally different from graph-search planners — explores the configuration space via random sampling. |
| 18 | Savitzky-Golay Path Smoother | `smoother_benchmark_node` | Path | Path | `e4_path_smoother` | Smooths a jagged planner output using Savitzky-Golay polynomial fitting. Pure Path→Path, no costmap needed. |

### Control (3 benchmarks)

| # | Algorithm | Kernel Package | Input | Output | Session | Description |
|---|---|---|---|---|---|---|
| 20 | CHAMP Gait + IK | `e1_autonomous_quadruped` | Twist | JointTrajectory | `e4_quadruped_controller` | Converts velocity commands into 12-joint leg trajectories using a gait generator (stance/swing phases) and inverse kinematics for each leg. |
| 21 | ros2_control PID | `joint_trajectory_controller` | JointTrajectory | JointState | `e4_pid_controller` | Tracks desired joint positions by computing effort (torque) commands using PID control. The lowest-level controller before the hardware. |
| 22 | Diff Drive Controller | `diff_drive_controller` | Twist | Odometry | `e4_diff_drive_controller` | Converts Twist velocity commands into left/right wheel velocities for differential drive robots. Forked with tracepoints, uses ros2_control with mock hardware. |

---

## Prerequisites

- ROS 2 Humble
- LTTng (`lttng-tools`, `lttng-ust-dev`, `liblttng-ust-dev`)
- `babeltrace` (for trace inspection)
- Docker container with the full benchmark workspace

---

## 1. Build

```bash
cd /tmp/benchmark_ws

# Build traced kernels + benchmark components + tracetools
colcon build --packages-select \
  tracetools_benchmark \
  e1_autonomous_quadruped \
  e4_latentros \
  nav2_amcl \
  nav2_costmap_2d \
  robot_localization \
  nav2_planner \
  nav2_controller \
  joint_trajectory_controller \
  champ_base

source install/setup.bash
```

> **Note:** The traced kernel forks live in
> `src/benchmarks/benchmarks/other/traced_kernels/`. Colcon prefers workspace source
> packages over installed binaries, so the instrumented versions will be used at runtime.

---

## 2. Run a Trace (collect data)

Each benchmark has a `trace_<name>.launch.py` that:
- Starts an LTTng tracing session
- Plays the rosbag in a loop (`--loop`)
- Launches the input component, kernel, and output component

```bash
# Example: PointCloud → LaserScan
ros2 launch e4_latentros trace_pc_to_laserscan.launch.py
```

**Let it run** until you have enough data (target: ~5000 messages).

### Full list of trace commands

```bash
# --- Perception ---
ros2 launch e4_latentros trace_pc_to_laserscan.launch.py
ros2 launch e4_latentros trace_image_resize.launch.py
ros2 launch e4_latentros trace_depthimage_to_laserscan.launch.py
ros2 launch e4_latentros trace_stereo_disparity.launch.py
ros2 launch e4_latentros trace_depthimage_to_pointcloud.launch.py
ros2 launch e4_latentros trace_laserscan_range_filter.launch.py
ros2 launch e4_latentros trace_laserscan_median_filter.launch.py

# --- Mapping ---
ros2 launch e4_latentros trace_global_costmap.launch.py
ros2 launch e4_latentros trace_local_costmap.launch.py
ros2 launch e4_latentros trace_slam_toolbox.launch.py

# --- Localization ---
ros2 launch e4_latentros trace_amcl.launch.py
ros2 launch e4_latentros trace_base_to_footprint_ekf.launch.py
ros2 launch e4_latentros trace_footprint_to_odom_ekf.launch.py
ros2 launch e4_latentros trace_state_estimation.launch.py

# --- Planning ---
ros2 launch e4_latentros trace_global_planner.launch.py
ros2 launch e4_latentros trace_theta_star_planner.launch.py
ros2 launch e4_latentros trace_rrt_planner.launch.py
ros2 launch e4_latentros trace_path_smoother.launch.py

# --- Control ---
ros2 launch e4_latentros trace_quadruped_controller.launch.py
ros2 launch e4_latentros trace_pid_controller.launch.py
ros2 launch e4_latentros trace_diff_drive_controller.launch.py
```

---

## 3. Stop the Trace (flush LTTng buffers)

**Important:** LTTng buffers data in memory and only writes to disk on session stop.
You MUST stop the session before reading trace data.

```bash
# Ctrl+C the trace launch first, then:
lttng stop <session_name>
```

Example:
```bash
lttng stop e4_pc_to_laserscan
```

---

## 4. Verify with babeltrace (inspect raw trace data)

### Check event counts per tracepoint

```bash
babeltrace /root/.ros/tracing/e4_pc_to_laserscan 2>/dev/null \
  | grep robotperf \
  | awk -F'robotperf_benchmarks:' '{print $2}' \
  | awk -F':' '{print $1}' \
  | sort | uniq -c | sort -rn
```

Expected output (all counts should be approximately equal):
```
  5000 robotperf_pointcloud_input_cb_init
  5000 robotperf_pointcloud_input_cb_fini
  5000 robotperf_msg_received_1
  5000 robotperf_msg_published_1
  5000 robotperf_msg_published_2
  5000 robotperf_laserscan_input_cb_init
  5000 robotperf_laserscan_input_cb_fini
```

### Verify key propagation (same key across all tracepoints)

```bash
# Pick any key number and check it appears in all tracepoints
babeltrace /root/.ros/tracing/e4_pc_to_laserscan 2>/dev/null \
  | grep "key = 42"
```

You should see 6+ lines all with `key = 42`, confirming the key propagated
from input → kernel → output.

### View first few events

```bash
babeltrace /root/.ros/tracing/e4_pc_to_laserscan 2>/dev/null | head -20
```

### Count total samples while the trace is still running

LTTng buffers in memory, so you need a stop/start cycle:

```bash
# In a separate terminal while the trace launch is still running:
lttng stop <session_name>
babeltrace /root/.ros/tracing/<session_name> 2>/dev/null \
  | grep -c robotperf_msg_received_1
lttng start <session_name>
```

---

## 5. Analyze (compute T_in, T_kernel, T_out statistics)

Each benchmark has an `analyze_<name>.launch.py` that reads the LTTng trace,
matches the 6-tracepoint chain, and computes latency statistics.

```bash
# Run directly with Python for --warmup support
python3 src/benchmarks/benchmarks/end_to_end/e4_latentros/launch/analyze_pc_to_laserscan.launch.py \
  --trace_path /root/.ros/tracing/e4_pc_to_laserscan \
  --warmup 100
```

The `--warmup N` flag discards the first N matched message chains to skip
startup transients and measure only steady-state performance.

### Full list of analyze commands

```bash
P="src/benchmarks/benchmarks/end_to_end/e4_latentros/launch"

# --- Perception ---
python3 $P/analyze_pc_to_laserscan.launch.py         --trace_path /root/.ros/tracing/e4_pc_to_laserscan --warmup 100
python3 $P/analyze_image_resize.launch.py             --trace_path /root/.ros/tracing/e4_image_resize --warmup 100
python3 $P/analyze_depthimage_to_laserscan.launch.py  --trace_path /root/.ros/tracing/e4_depthimage_to_laserscan --warmup 100
python3 $P/analyze_stereo_disparity.launch.py         --trace_path /root/.ros/tracing/e4_stereo_disparity --warmup 100
python3 $P/analyze_depthimage_to_pointcloud.launch.py      --trace_path /root/.ros/tracing/e4_depthimage_to_pointcloud --warmup 100
python3 $P/analyze_laserscan_range_filter.launch.py   --trace_path /root/.ros/tracing/e4_laserscan_range_filter --warmup 100
python3 $P/analyze_laserscan_median_filter.launch.py  --trace_path /root/.ros/tracing/e4_laserscan_median_filter --warmup 100

# --- Mapping ---
python3 $P/analyze_global_costmap.launch.py           --trace_path /root/.ros/tracing/e4_global_costmap --warmup 100
python3 $P/analyze_local_costmap.launch.py            --trace_path /root/.ros/tracing/e4_local_costmap --warmup 100
python3 $P/analyze_slam_toolbox.launch.py             --trace_path /root/.ros/tracing/e4_slam_toolbox --warmup 100

# --- Localization ---
python3 $P/analyze_amcl.launch.py                     --trace_path /root/.ros/tracing/e4_amcl --warmup 100
python3 $P/analyze_base_to_footprint_ekf.launch.py    --trace_path /root/.ros/tracing/e4_base_to_footprint_ekf --warmup 100
python3 $P/analyze_footprint_to_odom_ekf.launch.py    --trace_path /root/.ros/tracing/e4_footprint_to_odom_ekf --warmup 100
python3 $P/analyze_state_estimation.launch.py         --trace_path /root/.ros/tracing/e4_state_estimation --warmup 100

# --- Planning ---
python3 $P/analyze_global_planner.launch.py           --trace_path /root/.ros/tracing/e4_global_planner --warmup 100
python3 $P/analyze_theta_star_planner.launch.py       --trace_path /root/.ros/tracing/e4_theta_star_planner --warmup 100
python3 $P/analyze_rrt_planner.launch.py              --trace_path /root/.ros/tracing/e4_rrt_planner --warmup 100
python3 $P/analyze_path_smoother.launch.py            --trace_path /root/.ros/tracing/e4_path_smoother --warmup 100

# --- Control ---
python3 $P/analyze_quadruped_controller.launch.py     --trace_path /root/.ros/tracing/e4_quadruped_controller --warmup 100
python3 $P/analyze_pid_controller.launch.py           --trace_path /root/.ros/tracing/e4_pid_controller --warmup 100
python3 $P/analyze_diff_drive_controller.launch.py    --trace_path /root/.ros/tracing/e4_diff_drive_controller --warmup 100
```

---

## 6. Full Workflow Example (end-to-end for one benchmark)

```bash
# === PointCloud to LaserScan ===
cd /tmp/benchmark_ws
source install/setup.bash

# 1. Clean any old trace
rm -rf /root/.ros/tracing/e4_pc_to_laserscan

# 2. Run the trace (rosbag loops automatically)
ros2 launch e4_latentros trace_pc_to_laserscan.launch.py
# ... wait for ~5000 messages (~16 rosbag loops at 324 msgs/loop) ...
# Ctrl+C

# 3. Stop LTTng to flush buffers to disk
lttng stop e4_pc_to_laserscan

# 4. Verify: check tracepoint counts
babeltrace /root/.ros/tracing/e4_pc_to_laserscan 2>/dev/null \
  | grep robotperf \
  | awk -F'robotperf_benchmarks:' '{print $2}' \
  | awk -F':' '{print $1}' \
  | sort | uniq -c | sort -rn

# 5. Verify: check key propagation
babeltrace /root/.ros/tracing/e4_pc_to_laserscan 2>/dev/null \
  | grep "key = 100"

# 6. Analyze (skip first 100 warmup samples)
python3 src/benchmarks/benchmarks/end_to_end/e4_latentros/launch/analyze_pc_to_laserscan.launch.py \
  --trace_path /root/.ros/tracing/e4_pc_to_laserscan \
  --warmup 100
```

---

## Special Cases

### Timer-driven kernels (EKF, State Estimation, Costmaps, DWB, PID)

These kernels publish on a fixed-rate timer, not per input message. The key is stored
from the most recent input callback and used at the next timer-triggered publish.
This means key correlation is **approximate** (the Nth output corresponds to whichever
input was most recently received). With 5000+ samples, aggregate statistics (mean, p99)
are still valid.

| Kernel | Input Rate | Output Rate | Notes |
|---|---|---|---|
| State Estimation | ~134 Hz (joint_states) | 50 Hz (timer) | Output rate < input rate |
| EKF (both) | ~100 Hz | 200 Hz (timer) | Output rate > input rate |
| Global Costmap | ~5 Hz (scan) | 10 Hz (publish_frequency) | Multiple scans per costmap |
| Local Costmap | ~5 Hz (scan) | 2 Hz (publish_frequency) | Multiple scans per costmap |
| DWB Controller | 1 path (action) | 20 Hz (controller_frequency) | Many outputs per input |
| PID Controller | ~200 Hz (trajectory) | 250 Hz (update_rate) | ~1:1 |

### Action-based kernels (Global Planner, DWB Controller)

These kernels receive goals via ROS 2 action interfaces, not topic subscriptions:

- **Global Planner**: `planner_benchmark_client.py` sends 5000 `ComputePathToPose`
  action goals. Each goal triggers one path computation.
- **DWB Controller**: `controller_benchmark_client.py` sends `FollowPath` action goals.
  DWB runs at 20 Hz, publishing `cmd_vel` each tick.

### AMCL forced updates

AMCL normally skips scans if the robot hasn't moved enough (`update_min_d=0.25`).
The trace launch overrides: `update_min_d=0.0, update_min_a=0.0` to force a particle
filter update on **every** scan, ensuring consistent 1:1 input→output for benchmarking.

### PID controller mock hardware

The PID benchmark uses `mock_components/GenericSystem` instead of Gazebo hardware
interfaces. The minimal URDF is in `config/go2_mock_hardware.urdf` with 12 joints.
The controller config is in `config/go2_ros_control_benchmark.yaml`.

---

## Forked Kernels (with tracepoints)

The following packages were forked from their upstream ROS 2 releases and instrumented
with `robotperf_msg_received_1` / `robotperf_msg_published_1` tracepoints plus key
propagation via `header.stamp.nanosec`:

| Package | Location | Key Propagation |
|---|---|---|
| `nav2_amcl` | `other/traced_kernels/nav2_amcl/` | `scan.nanosec` → stored → `pose.nanosec` |
| `robot_localization` | `other/traced_kernels/robot_localization/` | `msg.nanosec` → `latentros_key_stored_` → `filtered.nanosec` |
| `nav2_planner` | `other/traced_kernels/nav2_planner/` | `goal.header.nanosec` → `result.path.header.nanosec` |
| `nav2_controller` | `other/traced_kernels/nav2_controller/` | `path.header.nanosec` → `latentros_key_` → cmd_vel tracepoint |
| `nav2_costmap_2d` | `unitree-go2-ros2/nav2_costmap_2d/` (in-tree) | `scan.nanosec` → `g_latentros_costmap_key` global → costmap tracepoint |
| `champ_base` | `unitree-go2-ros2/champ/champ_base/` (in-tree) | `joints.nanosec` → `latentros_key_` → `odom.nanosec` |
| `joint_trajectory_controller` | `other/traced_kernels/joint_trajectory_controller/` | `traj.header.nanosec` → atomic `latentros_key_` → update() tracepoints |

The two e1 kernels (`pointcloud_to_laserscan` and `quadruped_controller`) were already
forked with tracepoints in the `e1_autonomous_quadruped` package.

---ros2 launch e4_latentros trace_theta_star_planner.launch.py

## Package Structure

```
e4_latentros/
├── config/
│   ├── ekf_base_to_footprint_benchmark.yaml
│   ├── ekf_footprint_to_odom_benchmark.yaml
│   ├── go2_mock_hardware.urdf
│   └── go2_ros_control_benchmark.yaml
├── include/e4_latentros/
│   ├── *_input_component.hpp    (5 input components)
│   └── *_output_component.hpp   (6 output components)
├── src/
│   ├── *_input_component.cpp    (5 input components)
│   └── *_output_component.cpp   (6 output components)
├── scripts/
│   ├── planner_benchmark_client.py
│   └── controller_benchmark_client.py
├── launch/
│   ├── trace_*.launch.py        (11 trace launch files)
│   └── analyze_*.launch.py      (11 analyze launch files)
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## Troubleshooting

### babeltrace returns empty output
LTTng buffers in memory. You must `lttng stop <session>` before reading.

### Tracepoint chain breaks (Unexpected event: restarting sequence)
An output component's `fini` tracepoint isn't firing. This happens when an e1
component's publisher has no downstream subscribers. The `trace_pc_to_laserscan`
and `trace_quadruped_controller` launches include dummy subscribers to fix this.

### LTTng session already exists
```bash
lttng destroy <session_name>
rm -rf /root/.ros/tracing/<session_name>
```

### Build fails: cannot find tracetools_benchmark
Build `tracetools_benchmark` first or ensure it's in your `colcon build` command.
All traced kernel `package.xml` files declare `<depend>tracetools_benchmark</depend>`.

### Lifecycle nodes don't activate (AMCL, costmaps, planner, controller)
The trace launch files include `nav2_lifecycle_manager` with `autostart: True`.
If nodes fail to activate, check that `map_server` started successfully and the
map file exists at the expected path in `go2_config`.
