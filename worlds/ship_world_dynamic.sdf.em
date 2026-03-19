@# -----------------
@# config variables 
@# -----------------
@{

# ----- ship parameters ----------------------------------- 

ship_size = (20.0,150.0,10.0) #width #length #height

ship_mass = 5000.0 #kg

ship_inertia = [
    [500.0, 0.0, 0.0],
    [0.0, 500.0, 0.0],
    [0.0, 0.0, 500.0]
] #kg*m^2

CoG_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

#---------------------------------------------------------
#  ------ corridor parameters ----------------------------


corridor_size = (6.305, 20.579, 3.677)
scale = tuple(a / b for a, b in zip(corridor_size, ship_size))


corridor_pose = (5.0, 40.0, 2.50, 0.0, 0.0, 0.0) # wrt the center of the ship (as the world)
bias =(0.078, 0.690, 0.445, 0.0, 0.0, 0.0)

corridor_pose = tuple(a+b for a,b in zip(corridor_pose,bias))

corridor_mass = 5000.0 #kg
corridor_inertia = [
    [500.0, 0.0, 0.0],
    [0.0, 500.0, 0.0],
    [0.0, 0.0, 500.0]
] #kg*m^2

mu = 1.0 
mu2 = 1.0 
#-------------------------------------------------------
# ----- motion parameters ------------------------------

roll_amplitude  = 0.0 #rad
roll_frequency  = 0.0 #Hz
roll_phase      = 0.0 #rad 

pitch_amplitude = 0.0
pitch_frequency = 0.0
pitch_phase     = 0.0

heave_amplitude = 0.0
heave_frequency = 0.0
heave_phase     = 0.0

roll_bound = 0.5 #rad 
pitch_bound = 0.2 #rad
heave_bound = 2.0 #meters

revolute_damping = 10.0 
revolute_friction = 5.0 

prismatic_damping = 10.0 
prismatic_friction = 5.0 
# ----------------------------------------------------
}@

<?xml version="1.0" ?> <!-- define the version (xml file)-->
<sdf version="1.8"> 

    <world name="ship_world"> <!-- define the world name-->
        <light type="directional" name="sun"> <!-- light of the world-->
            <cast_shadows>true</cast_shadows> 
            <pose>0 0 10 0 0 0</pose> <!-- position: 10 meters on z axis-->
            <diffuse>0.8 0.8 0.8 1</diffuse> <!-- diffuse light -->
            <specular>0.2 0.2 0.2 1</specular> <!-- specular reflex-->
            <direction>-0.5 0.1 -0.9</direction> <!-- direction of the light-->
        </light>

        <gravity>0 0 -9.81</gravity> <!-- add gravity (9.81 in z direction)-->

        <physics name="bullet_physics" type="bullet">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1.0</real_time_factor>
            <bullet>
                <solver>
                    <type>sequential_impulse</type>
                </solver>
            </bullet>
        </physics>

        <!--inside the world we can add multiple models-->
        <model name="ship_corridor_dynamic"> <!--define the model ship_corridor-->
            <static>false</static> <!-- static false because we want the world subjec to physics-->
            
            <!-- 
            !! kinematic chain of the corridor !! 

            4 link connected by joints, to simulate the dynamics of the ship
            world - fixed
                -> world_base_link
                    ->revolute ROLL (around x)
                        ->roll_link
                            ->revolute PITCH (around y)
                                ->pitch_link
                                    ->prismatic HEAVE (along z)
                                        ->CoG_link (Center_of_Gravity) [ship - Center of Rotation]
                                            ->fixed joint
                                                ->corridor_link 
            -->

            <!-- define the world base link (fixed wrt the world)-->
            <link name="world_base_link"> 
                <pose>0 0 0 0 0 0 </pose>
                <inertial> <!--every link should have an inertia, even if it's a reference link-->
                    <mass>1.0</mass> <!-- we set its mass to one-->
                    <inertia>
                        <ixx>0.1</ixx> 
                        <iyy>0.1</iyy>
                        <izz>0.1</izz>
                    </inertia>
                </inertial>
            </link>
            <joint name="world_fixed_joint" type="fixed">  <!-- fix the link (world_base_link) to the gazebo floor (world)-->
                <parent>world</parent>
                <child>world_base_link</child>
            </joint>

            <!-- link to simulate rolling of the CoG-->
            <link name="roll_link"> <!--every link should have an inertia, even if it's a reference link-->
                <inertial>
                    <mass>1.0</mass> 
                    <inertia>
                        <ixx>0.1</ixx> 
                        <iyy>0.1</iyy>
                        <izz>0.1</izz>
                    </inertia>
                </inertial>
            </link>

            <!-- create a joint between the world link and the roll link-->
            <!-- of the type: revolute, on x axis-->
            <joint name="roll_joint" type="revolute">
                <parent>world_base_link</parent> <!-- i-1 link connected to the joint (the fixed link)-->
                <child>roll_link</child> <!-- i link connected to the joint (the rotating link)--> 
                <axis>
                    <xyz>0 1 0</xyz> <!-- roll, revolute around y axis (X e Y are SWITCHED)-->
                    <limit> <!-- boundaries of roll (radiant)-->
                        <lower>-@(roll_bound)</lower> 
                        <upper>@(roll_bound)</upper>
                    </limit>
                    <dynamics><damping>@(revolute_damping)</damping><friction>@(revolute_friction)</friction></dynamics>
                </axis>
            </joint>

            <link name="pitch_link">
                <inertial>
                    <mass>1.0</mass>
                    <inertia>
                        <ixx>0.1</ixx>
                        <iyy>0.1</iyy>
                        <izz>0.1</izz>
                    </inertia>
                </inertial>
            </link>

            <joint name="pitch_joint" type="revolute">
                <parent>roll_link</parent>
                <child>pitch_link</child>
                <axis>
                    <xyz>1 0 0</xyz> <!-- pitch, revolute around x axis (X e Y are SWITCHED!)-->
                    <limit> <!-- boundaries of pitch (radiant)-->
                        <lower>-@(pitch_bound)</lower> 
                        <upper>@(pitch_bound)</upper>
                    </limit>
                    <dynamics><damping>@(revolute_damping)</damping><friction>@(revolute_friction)</friction></dynamics>
                </axis>
            </joint>

            <link name="CoG_link">
                <pose>@(CoG_pose[0]) @(CoG_pose[1]) @(CoG_pose[2]) @(CoG_pose[3]) @(CoG_pose[4]) @(CoG_pose[5])</pose> <!-- CoG pose-->
                <inertial>
                    <mass>@(ship_mass)</mass> <!-- mass of the ship-->
                    <inertia>
                        <ixx>@(ship_inertia[0][0])</ixx> <!-- inertial matrix of the ship-->
                        <iyy>@(ship_inertia[1][1])</iyy>
                        <izz>@(ship_inertia[2][2])</izz>
                    </inertia>
                </inertial>

            <!-- box simulating the ship-->
                <visual name ="ship_hull">
                    <geometry>
                        <box>
                            <size>@(ship_size[0]) @(ship_size[1]) @(ship_size[2])</size> <!-- dimension X Y Z-->
                        </box>
                    </geometry>
                    <material>
                        <ambient>0.5 0.5 0.8 0.3</ambient> <!-- semi transparent blu-->
                        <diffuse>0.5 0.5 0.8 0.3</diffuse>
                    </material>
                </visual>
            <!-- //////////// -->
            </link>

            <joint name="heave_joint" type="prismatic">
                <parent>pitch_link</parent>
                <child>CoG_link</child>
                <axis>
                    <xyz>0 0 1</xyz>
                    <limit>
                        <lower>-@(heave_bound)</lower>
                        <upper>@(heave_bound)</upper>
                    </limit>
                    <dynamics><damping>@(prismatic_damping)</damping><friction>@(prismatic_friction)</friction></dynamics>
                </axis>
            </joint>
            
            <link name="corridor_link"> 
                <pose>@(corridor_pose[0]) @(corridor_pose[1]) @(corridor_pose[2]) @(corridor_pose[3]) @(corridor_pose[4]) @(corridor_pose[5])</pose>
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
                                <mu>@(mu)</mu> <mu2>@(mu2)</mu2>
                            </ode>
                        </friction>
                    </surface> 
                </collision>
            </link>

            <joint name="corridor_joint" type="fixed">
                <parent>CoG_link</parent>
                <child>corridor_link</child>
            </joint>

            <!--CMake aggiunge automaticamente il prefisso lib al nome ShipMotionPlugin-->
            <!--name="ship_gazebo::ShipMotionPlugin" deve corrispondere alla stringa in GZ_ADD_PLUGIN_ALIAS nel .cc-->
            <plugin 
                filename="libShipMotionPlugin.so" 
                name="ship_gazebo::ShipMotionPlugin">

                <!--roll-->
                <roll_amplitude>@(roll_amplitude)</roll_amplitude>
                <roll_frequency>@(roll_frequency)</roll_frequency>
                <roll_phase>@(roll_phase)</roll_phase>

                <!--pitch-->
                <pitch_amplitude>@(pitch_amplitude)</pitch_amplitude>
                <pitch_frequency>@(pitch_frequency)</pitch_frequency>
                <pitch_phase>@(pitch_phase)</pitch_phase>

                <!--heave-->
                <heave_amplitude>@(heave_amplitude)</heave_amplitude>
                <heave_frequency>@(heave_frequency)</heave_frequency>
                <heave_phase>@(heave_phase)</heave_phase>
            </plugin>

        </model>

    </world>
</sdf>