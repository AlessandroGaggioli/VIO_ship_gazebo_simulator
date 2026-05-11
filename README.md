# VIO Ship Gazebo Simulator

A comprehensive ROS 2 + Gazebo simulator for visual-inertial odometry (VIO) testing on a TurtleBot3 robot aboard a dynamic vessel. The system includes real-time ship motion (roll, pitch, heave), stereo vision pipeline, IMU compensation, sensor data recording, and SLAM/odometry strategies.

---

## System Overview

### What is this system?

The VIO Ship Gazebo Simulator recreates a **TurtleBot3 Waffle robot navigating inside a ship corridor during active sea motion**. The vessel undergoes realistic sinusoidal roll, pitch, and heave oscillations, and the simulator captures:

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


