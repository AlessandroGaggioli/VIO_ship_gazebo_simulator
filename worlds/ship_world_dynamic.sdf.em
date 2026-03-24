@# -----------------
@# config variables
@# -----------------
@{

# ----- ship parameters -----------------------------------
ship_size = (20.0, 150.0, 10.0)  # width, length, height [m]
ship_mass = 6000.0               # kg
ship_inertia = [
    [6000.0, 0.0,     0.0    ],
    [0.0,     6000.0, 0.0    ],
    [0.0,     0.0,     6000.0]
]  # kg·m²

CoG_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

# ----- link intermedi (heave, roll, pitch) ---------------
# Masse sufficientemente grandi da evitare instabilità numerica in DART,
# ma trascurabili rispetto alla massa principale di 6000 kg.
intermediate_mass = 10.0 # kg
intermediate_ixx  = 1.0  # kg·m²
intermediate_iyy  = 1.0
intermediate_izz  = 1.0

# ----- corridor parameters --------------------------------
corridor_size = (6.305, 20.579, 3.677)
scale = tuple(a / b for a, b in zip(corridor_size, ship_size))
corridor_pose = (5.0, 40.0, 2.50, 0.0, 0.0, 0.0)
bias = (0.078, 0.690, 0.445, 0.0, 0.0, 0.0)
corridor_pose = tuple(a + b for a, b in zip(corridor_pose, bias))
corridor_mass = 500.0  # kg
corridor_inertia = [
    [500.0, 0.0,   0.0  ],
    [0.0,   500.0, 0.0  ],
    [0.0,   0.0,   500.0]
]  # kg·m²

# attrito alto per impedire scivolamento del robot durante le oscillazione
mu  = 8.0
mu2 = 8.0

# ----- robot parameters -----------------------------------
robot_x     = corridor_pose[0]
robot_y     = corridor_pose[1] - (corridor_size[1] * scale[1] / 2.0) + 0.30
robot_z     = corridor_pose[2] - (corridor_size[2] * scale[2] / 2.0) + 0.35
robot_roll  = corridor_pose[3]
robot_pitch = corridor_pose[4]
robot_yaw   = corridor_pose[5] + 90 * 0.0174533333
robot_pose  = (robot_x, robot_y, robot_z, robot_roll, robot_pitch, robot_yaw)

# --------- obtacles --------------------------------
ob_x = corridor_pose[0] - (corridor_size[0] * scale[1] / 2.0) + 0.75
ob_y = corridor_pose[1] - (corridor_size[1] * scale[1] / 2.0) + 1.00
ob_z = corridor_pose[2] - (corridor_size[2] * scale[2] / 2.0) + 0.45

ob1_pose = (ob_x,ob_y,ob_z,0,0,0) 

ob_x = corridor_pose[0] - (corridor_size[0] * scale[1] / 2.0) + 0.25
ob_y = corridor_pose[1] - (corridor_size[1] * scale[1] / 2.0) + 1.20
ob_z = corridor_pose[2] - (corridor_size[2] * scale[2] / 2.0) + 0.45

ob2_pose = (ob_x,ob_y,ob_z,0,0,0)

# ----- motion parameters ----------------------------------
roll_amplitude  = 0.3   # rad
roll_frequency  = 0.1   # Hz
roll_phase      = 0.0   # rad

pitch_amplitude = 0.0
pitch_frequency = 0.0
pitch_phase     = 0.0

heave_amplitude = 0.0
heave_frequency = 0.0
heave_phase     = 0.0

roll_bound  = 0.5   # rad
pitch_bound = 0.0   # rad
heave_bound = 0.0   # m

revolute_damping  = 100.0
revolute_friction = 10.0

prismatic_damping  = 100.0
prismatic_friction = 10.0

# ----- IMU ship parameters --------------------------------
imu_ship_rate   = 100.0    # Hz
imu_noise_accel = 0.00186  # m/s²
imu_noise_gyro  = 0.00010  # rad/s
imu_bias_accel  = 0.0
imu_bias_gyro   = 0.0
}@

<?xml version="1.0" ?>
<sdf version="1.8">

    <world name="ship_world">

        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <direction>-0.5 0.1 -0.9</direction>
        </light>

        <gravity>0 0 -9.81</gravity>

        <physics name="default_physics" type="dart">
            <max_step_size>0.001</max_step_size>
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

            <!-- HEAVE - traslazione lungo Z-->
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

            <!-- ROLL - rotazione attorno a Y -->

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
                    <xyz>0 1 0</xyz>
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

            <!-- PITCH - rotazione attorno a X -->
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
                    <xyz>1 0 0</xyz>
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
            
            <!-- CoG - massa e IMU nave -->
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
                        <box>
                            <size>@(ship_size[0]) @(ship_size[1]) @(ship_size[2])</size>
                        </box>
                    </geometry>
                    <material>
                        <ambient>0.5 0.5 0.8 0.3</ambient>
                        <diffuse>0.5 0.5 0.8 0.3</diffuse>
                    </material>
                </visual>

                <!-- IMU nave: pubblicata su /ship/imu (bridge ROS2) -->
                <sensor name="ship_imu" type="imu">
                    <pose>0 0 0 0 0 0</pose>
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

            <link name="corridor_link">
                <pose>@(corridor_pose[0]) @(corridor_pose[1]) @(corridor_pose[2]) @(corridor_pose[3]) @(corridor_pose[4]) @(corridor_pose[5])</pose>
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
                    <geometry>
                        <mesh>
                            <uri>file:///home/alienware/ship_ws/src/ship_gazebo/models/ship_corridor.dae</uri>
                            <scale>@(scale[0]) @(scale[1]) @(scale[2])</scale>
                        </mesh>
                    </geometry>
                </visual>

                <collision name="corridor_collision">
                    <geometry>
                        <mesh>
                            <uri>file:///home/alienware/ship_ws/src/ship_gazebo/models/ship_corridor.stl</uri>
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
            </plugin>

            <!--===============OBSTACLE 1=======================================-->
            <link name="obstacle_1_link">
                <pose>@(ob1_pose[0]) @(ob1_pose[1]) @(ob1_pose[2]) 0 0 0</pose>
                <gravity>false</gravity>

                <collision name="collision">
                    <geometry>
                        <box><size>0.2 0.2 0.2</size></box>
                    </geometry>
                </collision>

                <visual name="visual">
                    <geometry>
                        <box><size>0.2 0.2 0.2</size></box>
                    </geometry>
                    <material>
                        <ambient>1 0 0 1</ambient>
                        <diffuse>1 0 0 1</diffuse>
                    </material>
                </visual>
            </link>

            <joint name="obstacle_1_joint" type="fixed">
                <parent>corridor_link</parent>
                <child>obstacle_1_link</child>
            </joint>
            <!-- ===============================================================-->

            <!--===============OBSTACLE 2=======================================-->
            <link name="obstacle_2_link">
                <pose>@(ob2_pose[0]) @(ob2_pose[1]) @(ob2_pose[2]) 0 0 0</pose>
                <gravity>false</gravity>

                <collision name="collision">
                    <geometry>
                        <cylinder>
                            <radius>0.2</radius>
                            <length>0.2</length>
                        </cylinder>
                    </geometry>
                </collision>

                <visual name="visual">
                    <geometry>
                        <cylinder>
                            <radius>0.2</radius>
                            <length>0.2</length>
                        </cylinder>
                    </geometry>
                    <material>
                        <ambient>0 1 0 1</ambient>
                        <diffuse>0 1 0 1</diffuse>
                    </material>
                </visual>
            </link>

            <joint name="obstacle_2_joint" type="fixed">
                <parent>corridor_link</parent>
                <child>obstacle_2_link</child>
            </joint>
            <!-- ===============================================================-->

        </model>

        <include>
            <uri>file:///home/alienware/ship_ws/src/ship_gazebo/models/turtlebot3_stereo</uri>
            <name>turtlebot3_waffle</name>
            <pose>@(robot_pose[0]) @(robot_pose[1]) @(robot_pose[2]) @(robot_pose[3]) @(robot_pose[4]) @(robot_pose[5])</pose>
        </include>

        <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
        <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
        <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
        <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
            <render_engine>ogre2</render_engine>
        </plugin>
        <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    </world>
</sdf>