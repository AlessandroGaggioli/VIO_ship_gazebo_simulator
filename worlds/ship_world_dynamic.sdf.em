<?xml version="1.0" ?>
@# -----------------
@# config variables
@# -----------------
@{

import os
import yaml

from ament_index_python.packages import get_package_share_directory

_pkg_path = get_package_share_directory('ship_gazebo')
_cfg_path = os.path.join(_pkg_path, 'config', 'world_objects.yaml')

with open(_cfg_path) as f:
    _objects = yaml.safe_load(f)

#==========================================================
# ----- ship parameters -----------------------------------
#==========================================================

ship_size = (141.0, 20.16, 26.7)  # width, length, height [m]
CoG_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

ship_mass = 1000.0  
ship_inertia = [
    [1000.0, 0.0,     0.0    ],
    [0.0,     1000.0, 0.0    ],
    [0.0,     0.0,     1000.0]
]

#==========================================================
# ----- link intermedi (heave, roll, pitch) ---------------
#==========================================================

intermediate_mass = 10 
intermediate_ixx  = 10  
intermediate_iyy  = 10
intermediate_izz  = 10

#==========================================================
# ----- corridor parameters -------------------------------
#==========================================================

#corridor_size = (20.579, 6.305, 3.677) #one corridor 
corridor_size = (61.737, 8.803, 3.300) #three corridor (large)

scale = tuple(a / b for a, b in zip(corridor_size, ship_size))
scale = (scale[0], scale[1]*1.2, scale[2] * 4) + (0.0,0.0,0.0)

target_center = (40.0, 5.0, 2.50, 0.0, 0.0, 0.0)

#stl_center = (-0.690,0.078,0.445,0.0,0.0,0.0) #one corridor center stl 
stl_center = (-0.036, -0.003, -0.255, 0.0, 0.0, 0.0) #three corridor center stl

stl_center_scaled = tuple(a * b for a, b in zip(stl_center, scale))

corridor_pose = tuple(a - b for a, b in zip(target_center, stl_center_scaled)) 
floor_distance = (1.7996-0.005) * scale[2]

corridor_mass =10 
corridor_inertia = [
    [10, 0.0,   0.0  ],
    [0.0,   10, 0.0  ],
    [0.0,   0.0,   10]
]  

# attrito alto per impedire scivolamento del robot durante le oscillazione
mu  = 1.2
mu2 = 1.2

#===========================================================
# ----- counter weight to balance for ship control ---------
#===========================================================
#total mass (corridor+robot)
total_payload_mass = corridor_mass + 1.37 
cw_pose_str = "{} {} {} 0 0 0".format(-target_center[0], -target_center[1], -target_center[2])

#==========================================================
# --------- obtacles --------------------------------------
#==========================================================

corridor_origin_x = - (corridor_size[0] * scale[0] / 2.0)

def _compute_pose(item,z_half=0.0):
    ox = corridor_origin_x + item['offset'][0]
    oy = item['offset'][1]
    oz = -floor_distance + z_half + item['offset'][2]
    return (ox, oy, oz)

for ob in _objects.get('obstacles', []):
    z_half = ob['size'][2]/2.0 if ob['type']=='box' else ob['length']/2.0
    ob['pose'] = _compute_pose(ob, z_half)

#===========================================================
# ----- robot parameters -----------------------------------
#===========================================================

robot_mass = 1.37 
robot_size = (0.354, 0.178, 0.144)

robot_x     = - (corridor_size[0] * scale[0] / 2.0) + (robot_size[0] / 2.0) + 1.7 
robot_y     = 0.0 
robot_z     = - floor_distance + 0.01
robot_roll  = 0.0
robot_pitch = 0.0
robot_yaw   = 0.0
robot_pose  = (robot_x, robot_y, robot_z, robot_roll, robot_pitch, robot_yaw)

for m in _objects.get('models', []):
    if m['offset'] is None:
        m['pose'] = robot_pose   # usa la posa calcolata
    else:
        m['pose'] = _compute_pose(m)

#===========================================================
# ----- motion parameters ----------------------------------
#==========================================================

if 'roll_amplitude' not in locals():
    roll_amplitude  = 0.3   # rad
if 'pitch_amplitude' not in locals():
    pitch_amplitude = 0.2   # rad
if 'heave_amplitude' not in locals():
    heave_amplitude = 0.1   # rad

roll_frequency  = 0.1   # Hz
roll_phase      = 0.0   # rad

pitch_frequency = 0.1
pitch_phase     = 0.0

heave_frequency = 0.1
heave_phase     = 0.0

roll_bound  = 0.31   # rad
pitch_bound = 0.21   # rad
heave_bound = 0.11   # m

revolute_damping  = 100.0
revolute_friction = 10.0

prismatic_damping  = 100.0
prismatic_friction = 10.0

max_step_size = 0.0005

#===========================================================
# ----- IMU ship parameters --------------------------------
#===========================================================

imu_ship_rate   = 100.0    # Hz
imu_noise_accel = 0.00186  # m/s^2
imu_noise_gyro  = 0.00010  # rad/s
imu_bias_accel  = 0.0
imu_bias_gyro   = 0.0

#===========================================================
# ----- cameras parameters --------------------------------
#===========================================================
for cam in _objects.get('cameras', []):
    off = cam.get('offset', [0, 0, 0, 0, 0, 0])
    ref = cam.get('reference', 'absolute')

    if ref == 'start':
        x_final = corridor_origin_x + off[0]
    elif ref == 'end':
        x_final = corridor_end_x + off[0]
    elif ref in ['center', 'absolute']:
        x_final = off[0]
    else:
        x_final = off[0]

    cam['pose'] = (x_final, off[1], off[2], off[3], off[4], off[5])

}@
<sdf version="1.8">

    <world name="ship_world">

        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 50 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <direction>-0.5 0.1 -1</direction>
        </light>

        <scene>
            <grid>false</grid>
            <origin_visual>true</origin_visual>
            <ambient>0.4 0.4 0.4 1</ambient> 
        </scene>

        <!--================================================================-->

        <gravity>0 0 -9.81</gravity>

        <physics name="default_physics" type="dart">
            <max_step_size>@(max_step_size)</max_step_size>
            <real_time_factor>1.0</real_time_factor>
        </physics>

        <!-- 
        Modello nave: 
        world -> heave_joint (z) -> roll_joint(y) -> pitch_joint(x) -> CoG -> corridor
        -->

        <model name="ship_corridor_dynamic">
            <static>false</static>

            <link name="world_base_link">
                <pose>0 0 0 0 0 0</pose>
                <inertial>
                    <mass>1.0</mass>
                    <inertia>
                        <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
                    </inertia>
                </inertial>
            </link>

            <joint name="world_fixed_joint" type="fixed">
                <parent>world</parent>
                <child>world_base_link</child>
            </joint>

            <!--===============================-->
            <!-- HEAVE - traslazione lungo Z-->
            <!--===============================-->
            <link name="heave_link">
                <pose>0 0 0 0 0 0</pose>  
                <gravity>false</gravity>
                <inertial>
                    <mass>@(intermediate_mass)</mass>
                    <inertia>
                        <ixx>@(intermediate_ixx)</ixx> <iyy>@(intermediate_iyy)</iyy> <izz>@(intermediate_izz)</izz>
                    </inertia>
                </inertial>
            </link>

            <joint name="heave_joint" type="prismatic">
                <parent>world_base_link</parent>
                <child>heave_link</child>
                <axis>
                    <xyz>0 0 1</xyz>
                    <limit>
                        <lower>-@(heave_bound)</lower>
                        <upper>@(heave_bound)</upper>
                        <effort>10000000000</effort> </limit>
                    <dynamics>
                        <damping>@(prismatic_damping)</damping>
                        <friction>@(prismatic_friction)</friction>
                    </dynamics>
                </axis>
            </joint>

            <!--===============================-->
            <!-- ROLL - rotazione attorno a Y -->
            <!--===============================-->

            <link name="roll_link">
                <pose>0 0 0 0 0 0</pose>
                <gravity>false</gravity>
                <inertial>
                    <mass>@(intermediate_mass)</mass>
                    <inertia>
                        <ixx>@(intermediate_ixx)</ixx> <iyy>@(intermediate_iyy)</iyy> <izz>@(intermediate_izz)</izz>
                    </inertia>
                </inertial>
            </link>

            <joint name="roll_joint" type="revolute">
                <parent>heave_link</parent>
                <child>roll_link</child>
                <axis>
                    <xyz>1 0 0</xyz>
                    <limit>
                        <lower>-@(roll_bound)</lower>
                        <upper>@(roll_bound)</upper>
                        <effort>10000000000</effort> </limit>
                    <dynamics>
                        <damping>@(revolute_damping)</damping>
                        <friction>@(revolute_friction)</friction>
                    </dynamics>
                </axis>
            </joint>

            <!--===============================-->
            <!-- PITCH - rotazione attorno a X -->
            <!--===============================-->

            <link name="pitch_link">
                <pose>0 0 0 0 0 0</pose>
                <gravity>false</gravity>
                <inertial>
                    <mass>@(intermediate_mass)</mass>
                    <inertia>
                        <ixx>@(intermediate_ixx)</ixx> <iyy>@(intermediate_iyy)</iyy> <izz>@(intermediate_izz)</izz>
                    </inertia>
                </inertial>
            </link>

            <joint name="pitch_joint" type="revolute">
                <parent>roll_link</parent>
                <child>pitch_link</child>
                <axis>
                    <xyz>0 1 0</xyz>
                    <limit>
                        <lower>-@(pitch_bound)</lower>
                        <upper>@(pitch_bound)</upper>
                        <effort>10000000000</effort> </limit>
                    <dynamics>
                        <damping>@(revolute_damping)</damping>
                        <friction>@(revolute_friction)</friction>
                    </dynamics>
                </axis>
            </joint>
            
            <!--===============================-->
            <!-- CoG - massa e IMU nave -->
            <!--===============================-->

            <link name="CoG_link">
                <pose>@(CoG_pose[0]) @(CoG_pose[1]) @(CoG_pose[2]) @(CoG_pose[3]) @(CoG_pose[4]) @(CoG_pose[5])</pose>
                <gravity>false</gravity>
                <inertial>
                    <mass>@(ship_mass)</mass>
                    <inertia>
                        <ixx>@(ship_inertia[0][0])</ixx>
                        <iyy>@(ship_inertia[1][1])</iyy>
                        <izz>@(ship_inertia[2][2])</izz>
                    </inertia>
                </inertial>

                <visual name="ship_hull">
                    <geometry>
                        <mesh>
                            <uri>file:///home/alienware/ship_ws/src/ship_gazebo/models/vessel_mode.dae</uri>
                        </mesh>
                    </geometry>

                    <!-- meno trasparente -->
                    <transparency>0.3</transparency>

                    <material>
                        <!-- grigio scuro -->
                        <ambient>0.2 0.2 0.2 1</ambient>
                        <diffuse>0.25 0.25 0.25 1</diffuse>
                        <specular>0.1 0.1 0.1 1</specular>
                        <emissive>0 0 0 1</emissive>
                    </material>
                </visual>

                <!-- IMU nave: pubblicata su /ship/imu (bridge ROS2) -->
                <sensor name="ship_imu" type="imu">
                    <pose relative_to='CoG_link'>0 0 0 0 0 0</pose>   
                    <always_on>true</always_on>
                    <update_rate>@(imu_ship_rate)</update_rate>
                    <visualize>false</visualize>
                    <imu>
                        <angular_velocity>
                            <x><noise type="gaussian"><mean>0.0</mean><stddev>@(imu_noise_gyro)</stddev><bias_mean>@(imu_bias_gyro)</bias_mean></noise></x>
                            <y><noise type="gaussian"><mean>0.0</mean><stddev>@(imu_noise_gyro)</stddev><bias_mean>@(imu_bias_gyro)</bias_mean></noise></y>
                            <z><noise type="gaussian"><mean>0.0</mean><stddev>@(imu_noise_gyro)</stddev><bias_mean>@(imu_bias_gyro)</bias_mean></noise></z>
                        </angular_velocity>
                        <linear_acceleration>
                            <x><noise type="gaussian"><mean>0.0</mean><stddev>@(imu_noise_accel)</stddev><bias_mean>@(imu_bias_accel)</bias_mean></noise></x>
                            <y><noise type="gaussian"><mean>0.0</mean><stddev>@(imu_noise_accel)</stddev><bias_mean>@(imu_bias_accel)</bias_mean></noise></y>
                            <z><noise type="gaussian"><mean>0.0</mean><stddev>@(imu_noise_accel)</stddev><bias_mean>@(imu_bias_accel)</bias_mean></noise></z>
                        </linear_acceleration>
                    </imu>
                </sensor>
            </link>

            <joint name="cog_fixed_joint" type="fixed">
                <parent>pitch_link</parent>
                <child>CoG_link</child>
            </joint>

            <!--===============================-->
            <!-- corridor -->
            <!--===============================-->

            <link name="corridor_link"> 
                <pose relative_to='CoG_link'>@(corridor_pose[0]) @(corridor_pose[1]) @(corridor_pose[2]) @(corridor_pose[3]) @(corridor_pose[4]) @(corridor_pose[5])</pose>
                <gravity>false</gravity>
                <inertial>
                    <mass>@(corridor_mass)</mass>
                    <inertia>
                        <ixx>@(corridor_inertia[0][0])</ixx>
                        <iyy>@(corridor_inertia[1][1])</iyy>
                        <izz>@(corridor_inertia[2][2])</izz>
                    </inertia>
                </inertial>

                <visual name="corridor_visual">
                    <pose>0 0 0.01 0 0 0</pose>  <!-- 2 mm -->
                    <geometry>
                        <mesh>
                            <uri>file:///home/alienware/ship_ws/src/ship_gazebo/models/ship_color/ship_large_corridor_colored_mix.dae</uri>
                            <scale>@(scale[0]) @(scale[1]) @(scale[2])</scale>
                        </mesh>
                    </geometry>
                    <!-- <ambient>1 1 1 1</ambient>
                    <diffuse>1 1 1 1</diffuse> -->
                    <cast_shadows>true</cast_shadows>
                </visual>

                <collision name="corridor_collision">
                    <geometry>
                        <mesh>
                            <uri>file:///home/alienware/ship_ws/src/ship_gazebo/models/ship_large_corridor.stl</uri>
                            <scale>@(scale[0]) @(scale[1]) @(scale[2])</scale>
                        </mesh>
                    </geometry>
                    <surface>
                        <contact>
                            <collide_bitmask>0xffff</collide_bitmask>
                        </contact>
                        <friction>
                            <ode>
                                <mu>@(mu)</mu>
                                <mu2>@(mu2)</mu2>
                            </ode>
                        </friction>
                    </surface>
                </collision>
            </link>

            

            <joint name="corridor_joint" type="fixed">
                <parent>CoG_link</parent>
                <child>corridor_link</child>
            </joint>

            <!--================================-->
            <!-- contrappeso per controllo nave -->
            <!--================================-->
            <link name="counterweight_link">
                <pose relative_to='CoG_link'>@(cw_pose_str)</pose>
                <gravity>false</gravity>
                <inertial>
                    <mass>@(total_payload_mass)</mass>
                    <inertia>
                        <ixx>1.0</ixx> <iyy>1.0</iyy> <izz>1.0</izz>
                    </inertia>
                </inertial>
                <visual name="counterweight_visual">
                    <geometry><sphere><radius>0.3</radius></sphere></geometry>
                    <material>
                        <ambient>1 0 0 0.5</ambient>
                        <diffuse>1 0 0 0.5</diffuse>
                    </material>
                </visual>
            </link>

            <joint name="counterweight_joint" type="fixed">
                <parent>CoG_link</parent>
                <child>counterweight_link</child>
            </joint>

            <!--==========================================-->
            <!-- CAMERAS -->
            <!--==========================================-->

            @[for cam in _objects.get('cameras', [])]@
            <link name="@(cam['name'])_link">
                <pose relative_to='@(cam['parent'])'>
                    @(cam['pose'][0]) @(cam['pose'][1]) @(cam['pose'][2])
                    @(cam['pose'][3]) @(cam['pose'][4]) @(cam['pose'][5])
                </pose>
                <gravity>false</gravity>
                <sensor name="@(cam['name'])" type="camera">
                    <pose>0 0 0 0 0 0</pose>
                    <always_on>true</always_on>
                    <update_rate>@(cam['update_rate'])</update_rate>
                    <camera>
                        <horizontal_fov>@(cam['fov'])</horizontal_fov>
                        <image>
                            <width>@(cam['width'])</width>
                            <height>@(cam['height'])</height>
                            <format>R8G8B8</format>
                        </image>
                        <clip><near>0.1</near><far>100</far></clip>
                    </camera>
                </sensor>
            </link>
            <joint name="@(cam['name'])_joint" type="fixed">
                <parent>@(cam['parent'])</parent>
                <child>@(cam['name'])_link</child>
            </joint>
            @[end for]@

            <!--===============================-->
            <!-- ==== Plugin ======-->
            <!--===============================-->

            <plugin filename="libShipMotionPlugin.so" name="ship_gazebo::ShipMotionPlugin">
                <roll_amplitude>@(roll_amplitude)</roll_amplitude>
                <roll_frequency>@(roll_frequency)</roll_frequency>
                <roll_phase>@(roll_phase)</roll_phase>

                <pitch_amplitude>@(pitch_amplitude)</pitch_amplitude>
                <pitch_frequency>@(pitch_frequency)</pitch_frequency>
                <pitch_phase>@(pitch_phase)</pitch_phase>

                <heave_amplitude>@(heave_amplitude)</heave_amplitude>
                <heave_frequency>@(heave_frequency)</heave_frequency>
                <heave_phase>@(heave_phase)</heave_phase>

                <max_step_size>@(max_step_size)</max_step_size>
            </plugin>

            <!--===============================-->
            <!-- OBSTACLES-->
            <!--===============================-->
            @[for ob in _objects.get('obstacles', [])]@
            <link name="@(ob['name'])_link">
                <pose relative_to='corridor_link'>
                    @(ob['pose'][0]) @(ob['pose'][1]) @(ob['pose'][2]) 0 0 0
                </pose>
                <gravity>false</gravity>

                @[if ob['type'] == 'box']@
                <collision name="collision"><geometry><box>
                    <size>@(ob['size'][0]) @(ob['size'][1]) @(ob['size'][2])</size>
                </box></geometry></collision>
                <visual name="visual"><geometry><box>
                    <size>@(ob['size'][0]) @(ob['size'][1]) @(ob['size'][2])</size>
                </box></geometry>
                <material>
                    <!-- <ambient>@(ob['color'][0]) @(ob['color'][1]) @(ob['color'][2]) 1</ambient>
                    <diffuse>@(ob['color'][0]) @(ob['color'][1]) @(ob['color'][2]) 1</diffuse> -->

                    <ambient>1 1 1 1</ambient>
                    <diffuse>1 1 1 1</diffuse>
                    <pbr>
                        <metal>
                        <albedo_map>file:///home/alienware/ship_ws/src/ship_gazebo/models/ship_color/texture_obs.png</albedo_map>
                        </metal>
                    </pbr>
                </material></visual>

                @[elif ob['type'] == 'cylinder']@
                <collision name="collision"><geometry><cylinder>
                    <radius>@(ob['radius'])</radius><length>@(ob['length'])</length>
                </cylinder></geometry></collision>
                <visual name="visual"><geometry><cylinder>
                    <radius>@(ob['radius'])</radius><length>@(ob['length'])</length>
                </cylinder></geometry>
                <material>
                    <!-- <ambient>@(ob['color'][0]) @(ob['color'][1]) @(ob['color'][2]) 1</ambient>
                    <diffuse>@(ob['color'][0]) @(ob['color'][1]) @(ob['color'][2]) 1</diffuse> -->
                    <ambient>1 1 1 1</ambient>
                    <diffuse>1 1 1 1</diffuse>
                    <pbr>
                        <metal>
                        <albedo_map>file:///home/alienware/ship_ws/src/ship_gazebo/models/ship_color/texture_obs.png</albedo_map>
                        </metal>
                    </pbr>
                </material></visual>

                @[end if]@
            </link>
            <joint name="@(ob['name'])_joint" type="fixed">
                <parent>corridor_link</parent>
                <child>@(ob['name'])_link</child>
            </joint>
            @[end for]@

            <light name="start_spot_light" type="spot">
                <pose relative_to="corridor_link">
                    @(corridor_origin_x + 2.5) 0 2.0 0 0 0
                </pose>
                <cast_shadows>true</cast_shadows>

                <diffuse>1 1 1 1</diffuse>

                <direction>0 0 -1</direction>

                <spot>
                    <inner_angle>0.3</inner_angle>
                    <outer_angle>0.8</outer_angle>
                    <falloff>0.5</falloff>
                </spot>

                <attenuation>
                    <range>8</range>
                    <constant>0.5</constant>
                    <linear>0.02</linear>
                    <quadratic>0.002</quadratic>
                </attenuation>
            </light>

            <!--===============================-->
            <!-- TURTLEBOT -->
            <!--===============================-->

            @[for m in _objects.get('models', [])]@
            <include>
                <uri>@(m['uri'])</uri>
                <name>@(m['name'])</name>
                <pose relative_to='@(m['parent'])'>
                    @(m['pose'][0]) @(m['pose'][1]) @(m['pose'][2])
                    @(m['pose'][3]) @(m['pose'][4]) @(m['pose'][5])
                </pose>
            </include>
            @[end for]@


        </model>
        <!--===============================-->
        <!--===============================-->

        <!--===============================-->
        <!-- Plugin-->
        <!--===============================-->

        <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
        <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
        <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
        <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
            <render_engine>ogre2</render_engine>
        </plugin>
        <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    </world>
</sdf>