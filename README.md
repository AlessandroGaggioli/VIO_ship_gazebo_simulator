# VIO Ship Gazebo Simulator

A comprehensive ROS 2 + Gazebo simulator for visual-inertial odometry (VIO) testing on a TurtleBot3 robot aboard a dynamic vessel. The system includes real-time ship motion (roll, pitch, heave), stereo vision pipeline, IMU compensation, sensor data recording, and SLAM/odometry strategies.

---

## System Overview

### What is this system?

The VIO Ship Gazebo Simulator recreates a **TurtleBot3 Waffle robot navigating inside a ship corridor during active sea motion**. 
The vessel undergoes realistic sinusoidal roll, pitch, and heave oscillations, and the simulator captures:

- **Stereo camera streams** (left/right rectified images) → pre-synchronized RGBD output
- **IMU data** (raw 6-DoF accelerometer + gyroscope) → motion-compensated for ship effects
- **Odometry** via stereo visual odometry and optional EKF fusion
- **SLAM** using RTAB-Map for mapping and loop detection
- **Ground truth** from Gazebo physics engine
- **TF transforms** for all sensor frames

### The World

The simulator loads `ship_world_dynamic.sdf`, which dynamically embeds:
- **Ship corridor environment** with navigable pathways
- **TurtleBot3 Waffle** equipped with stereo cameras + IMU
- **Dynamic motion plugin** that applies sinusoidal motion to the entire ship structure
  - Configurable **roll**, **pitch**, **heave** amplitudes
  - Real physics-based motion (not just visual tricks)

### The Robot

The TurtleBot3 is mounted with:
- **Stereo camera pair** (left @ 0.15m forward, 0.06m port; right @ 0.15m forward, 0.06m starboard)
- **IMU sensor** (offset 0.032m aft, 0.078m up from base)

---

## Workspace Setup

### 1. Clone and Install

```bash
mkdir -p ~/ship_ws/src
cd ~/ship_ws/src
git clone https://github.com/AlessandroGaggioli/VIO_ship_gazebo_simulator.git ship_gazebo

sudo apt update
sudo apt install ros-jazzy-ros-gz libgz-sim8-dev libgz-plugin2-dev
```

### 2. ROS2 Environment 

Add to ~/.bashrc : 
```bash
source /opt/ros/jazzy/setup.bash
source ~/ship_ws/install/setup.bash
```
then: 
```
source ~/.bashrc
```

### 3. Gazebo Plugin and Model Paths 

Add to ~/.bashrc : 
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:~/ship_ws/install/ship_gazebo/lib/ship_gazebo
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ship_ws/install/ship_gazebo/share/ship_gazebo/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ship_ws/install/ship_gazebo/share/ship_gazebo/worlds
source ~/.bashrc
```

### 4. GPU Acceleration for NVIDIA 

```bash
echo "export __NV_PRIME_RENDER_OFFLOAD=1" >> ~/.bashrc
echo "export __GLX_VENDOR_LIBRARY_NAME=nvidia" >> ~/.bashrc
echo "export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d10_nvidia.json" >> ~/.bashrc
source ~/.bashrc
```

### 5. Build 
```bash
cd ~/ship_ws
colcon build --symlink-install ship_gazebo
source install/setup.bash
```

---

## Core ROS2 Nodes

### Recording & Analysis Nodes

#### `odom_comparator.py`
Compares ground truth odometry vs estimated odometry during rosbag replay. 
Generates comprehensive PDF reports and CSV exports with:
- Position and orientation errors
- Linear and angular velocity errors
- Absolute Trajectory Error (ATE) metrics
- Segmented analysis by motion segments

**Parameters:**
- `gt_robot_topic`: Ground truth odometry topic (default: `/robot/ground_truth/odom`)
- `est_topic`: Estimated odometry topic (default: `stereo_odom`)
- `csv_output_path`: Path to save CSV data (empty = no save)
- `pdf_output_path`: Path to save PDF report (empty = no save)
- `use_sim_time`: Use simulation time (default: true)

#### `imu_comparator.py`
Compares raw IMU data vs compensated IMU data vs ship IMU. 
Skips the first 10 seconds of data to exclude initialization phase. 
Generates plots and CSV exports with synchronized IMU measurements.

**Parameters:**
- `csv_output_path`: Path to save CSV data (empty = no save)
- `pdf_output_path`: Path to save PDF report (empty = no save)
- `use_sim_time`: Use simulation time (default: true)

**Features:**
- Approximate time synchronization (±20ms tolerance)
- Quaternion to Euler angle conversion
- 10-second skip for stable initial conditions

#### `cmd_vel_pub.py`
Publishes pre-recorded velocity commands from a YAML trajectory file.

**Parameters:**
- `output_file`: Path to trajectory YAML file
- `use_sim_time`: Use simulation time (default: false)

#### `traj_recorder.py`
Records velocity commands from teleop into a YAML trajectory file for later replay.

**Parameters:**
- `output_file`: Path to save trajectory YAML
- `use_sim_time`: Use simulation time (default: false)

#### `imu_compensator.py`
Compensates raw IMU measurements for ship motion effects using accelerometer and gyroscope data.

---

## Launch Files

### `analyze_sim.launch.py`
Master launch file for analyzing rosbag recordings. 
Coordiantes `odom_comparator` and `imu_comparator` nodes with configurable output paths.

**Usage:**
```bash
ros2 launch ship_gazebo analyze_sim.launch.py \
    save_path:=<path> \
    filename:=<name> \
    save_odom:=<true/false> \
    save_imu:=<true/false> \
    est_topic:=<topic>
```

**Parameters:**
- `save_path`: Output directory (default: current directory)
- `filename`: Base filename for all outputs (default: `report`)
- `save_odom`: Save odometry CSV/PDF (default: false)
- `save_imu`: Save IMU CSV/PDF (default: false)
- `est_topic`: Estimated odometry topic (default: `stereo_odom`)

### `replay.launch.py`
Launches rosbag replay with configurable odometry type and compensation settings.

**Usage:**
```bash
ros2 launch ship_gazebo replay.launch \
    bag:=<bag_path> \
    odom_type:=<loosely/ekf> \
    comp:=<true/false> \
    3dof:=<true/false>
```

**Parameters:**
- `bag`: Path to rosbag file
- `odom_type`: Odometry fusion method (default: `loosely`)
- `comp`: Enable IMU compensation (default: true)
- `3dof`: Use 3-DOF mode instead of 6-DOF (default: false)

### `record.launch.py`
Launches recording pipeline for capturing rosbag data with optional motion.

---

## Plotting Tools

### `plot_odom_csv.py`
Offline plotting tool for odometry CSV files. 
Generates 6 subplots showing position/orientation errors and velocity errors.

**Usage:**
```bash
python3 plot_odom_csv.py <csv_file>
```

### `plot_imu_csv.py`
Offline plotting tool for IMU CSV files. 
Generates 6 subplots (2 rows × 3 columns) showing linear acceleration and angular velocity for each axis.

**Usage:**
```bash
python3 plot_imu_csv.py <csv_file>
```

---

## Complete Workflow

### Recording Phase

**Terminal 1: Start simulation with recording**
```bash
ros2 launch ship_gazebo record.launch motion:=true bag:=bag_name
```

**Terminal 2: Send velocity commands (choose one method)**

Option A - Use pre-recorded trajectory:
```bash
ros2 run ship_gazebo cmd_vel_pub.py --ros-args \
    -p output_file:=traj.yaml \
    -p use_sim_time:=true
```

Option B - Use teleop keyboard:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true
```

Option C - Record teleop trajectory for later replay:
```bash
ros2 run ship_gazebo traj_recorder --ros-args \
    -p output_file:=traj.yaml \
    -p use_sim_time:=true
```

### Analysis Phase

**Terminal 1: Launch analysis nodes**
```bash
ros2 launch ship_gazebo analyze_sim.launch.py \
    save_path:=/home/alienware/Gaggioli/SimulationResults/motion_line/comp_on_2d_off \
    filename:=filename \
    save_odom:=true \
    save_imu:=true \
    save_map:=true
```

**Terminal 2: Replay rosbag**
```bash
ros2 launch ship_gazebo replay.launch \
    bag:=bag_name \
    odom_type:=loosely \
    comp:=true \
    3dof:=false
```

**Output Files Generated:**
- `filename_odom.csv` - Odometry comparison data
- `filename_odom.pdf` - Odometry analysis report
- `filename_imu.csv` - IMU comparison data
- `filename_imu.pdf` - IMU analysis report
- `filename_map.pgm` - map into ~/ship_ws/maps folder 
- 'maps_metrics.csv` - write metrics of the last map generated

### Post-Analysis Visualization

**Plot saved odometry data:**
```bash
python3 ~/ship_ws/src/ship_gazebo/tools/plot_odom_csv.py \
    /path/filename_odom.csv
```

**Plot saved IMU data:**
```bash
python3 ~/ship_ws/src/ship_gazebo/tools/plot_imu_csv.py \
    /path/filename_imu.csv
```

---

## Notes

- Set `save_odom:=false` or `save_imu:=false` or `save_map:=false` to skip saving CSV/PDF files and only print results to terminal
- Always use `use_sim_time:=true` when running nodes with simulated data
- The first 10 seconds of IMU data are skipped to allow system stabilization
- Odometry analysis automatically starts when motion is detected and stops after 5 seconds of no motion


