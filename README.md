# VIO Ship Gazebo Simulator


# Create workspace and clone the package

```bash
mkdir -p ~/ship_ws/src
cd ~/ship_ws/src
git clone [https://github.com/AlessandroGaggioli/VIO_ship_gazebo_simulator.git](https://github.com/AlessandroGaggioli/VIO_ship_gazebo_simulator.git) ship_gazebo

sudo apt update
sudo apt install ros-jazzy-ros-gz libgz-sim8-dev libgz-plugin2-dev
```
# Bash configuration

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ship_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
# Create package

```bash
cd ~/ship_ws/src
ros2 pkg create --build-type ament_cmake ship_gazebo
```
# Plugin Path
To ensure Gazebo finds the plugin and 3D models correctly, add these lines to your ~/.bashrc file:

```bash
echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:~/ship_ws/install/ship_gazebo/lib/ship_gazebo" >> ~/.bashrc
```
# Models & Worlds Path
```bash
echo "export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ship_ws/install/ship_gazebo/share/ship_gazebo/models:~/ship_ws/install/ship_gazebo/share/ship_gazebo/worlds" >> ~/.bashrc
source ~/.bashrc
```

# Note for NVIDIA
It's safe to force the execution by GPU: 
```bash
	cd ~/ship_ws
	
	echo "export __NV_PRIME_RENDER_OFFLOAD=1" >> ~/.bashrc
	echo "export __GLX_VENDOR_LIBRARY_NAME=nvidia" >> ~/.bashrc
	echo "export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d10_nvidia.json" >> ~/.bashrc
	source ~/.bashrc
```

# Path Plugin Gazebo export
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:~/ship_ws/install/ship_gazebo/lib/ship_gazebo
source ~/.bashrc
```

# Physics Engine Configuration (Bullet)

The plugin is optimized for Bullet. Create the configuration file if it doesn't exist:
```bash
    mkdir -p ~/.gz/sim/8/
    cat > ~/.gz/sim/8/server.config << 'EOF'
    <server>
    <physics name="bullet_physics" type="bullet">
        <max_step_size>0.001</max_step_size>
        <real_time_factor>1.0</real_time_factor>
    </physics>
    </server>
    EOF
```

# IntelliSense Configuration (VSCode)
To avoid "include" errors in VSCode, create the .vscode/c_cpp_properties.json file in your workspace:
```bash
mkdir -p ~/ship_ws/.vscode

	cat > ~/ship_ws/.vscode/c_cpp_properties.json << 'EOF'
	{
	  "configurations": [
	    {
	      "name": "Linux",
	      "includePath": [
		"/home/alienware/ship_ws/**",
		"/usr/include/gz/sim8/**",
		"/usr/include/gz/plugin2/**",
		"/usr/include/gz/**",
		"/usr/include/**"
	      ],
	      "compilerPath": "/usr/bin/g++",
	      "cStandard": "c17",
	      "cppStandard": "c++17",
	      "intelliSenseMode": "linux-gcc-x64"
	    }
	  ],
	  "version": 4
	}
	EOF
```

Launch VSCode
```bash
	cd ~/ship_ws
	code . 
```

# Build and Compile 

```bash
cd ~/ship_ws
colcon build --packages-select ship_gazebo
source install/setup.bash
```

# Execution
To run the simulation: 
```bash
    ros2 launch ship_gazebo ship_sim.launch.py
```