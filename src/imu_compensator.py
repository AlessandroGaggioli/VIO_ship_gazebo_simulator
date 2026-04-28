#!/usr/bin/env python3

"""
This node subscribes to the raw IMU data of the ship and the robot, as well as the robot's odometry, 
to compute a compensated IMU message for the robot that accounts for the ship's motion (including tangential, centrifugal, and Coriolis accelerations).
The compensated IMU data is published on the /robot/imu/compensated topic.
The compensation can be enabled or disabled via a ROS parameter.

The compensation is based on the following equations:
- Apparent acceleration (acc_app) = tangential acceleration + centrifugal acceleration + Coriolis acceleration
- Compensated acceleration (acc_comp) = a_raw - R_s^r * (a_ship + acc_app) + R_s^r * g
- Compensated angular velocity (omega_comp) = omega_raw - R_s^r * omega_ship
Where:
- a_raw and omega_raw are the raw linear acceleration and angular velocity from the robot's IMU
- a_ship and omega_ship are the linear acceleration and angular velocity of the ship
- R_s^r is the rotation matrix from the ship's frame to the robot's frame
- g is the gravity vector (0, 0, 9.81 m/s^2) 

The synchronization of the data is handled by using the timestamps of the incoming messages, and the compensation is computed in real-time as new data arrives.
qos_profile_sensor_data is used for the subscriptions to ensure that we get the most recent data without worrying about message history, which is important for real-time compensation.

How to enable/disable compensation:
- The compensation can be enabled or disabled using the ROS parameter 'enable'.
- To enable compensation, set the parameter to True (default is True):
ros2 param set /imu_compensator enable true
- To disable compensation, set the parameter to False:
ros2 param set /imu_compensator enable false
When compensation is disabled, the node will simply pass through the raw IMU data from the robot without applying any corrections based on the ship's motion. 
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R 
from geometry_msgs.msg import Pose
import numpy as np
import message_filters


#=============================================
# data classes
#==============================================
"""
In this section, we define two data classes, ShipState and RobotState, 
to store the relevant information about the ship and the robot, respectively.
- ShipState: This class holds: 
    the latest IMU message from the ship, 
    the last timestamp when the ship's IMU data was received, 
    the current angular velocity (omega) of the ship, 
    the angular acceleration (omega_dot) of the ship, 
    and the linear acceleration of the ship.
- RobotState: This class holds:
    the linear velocity of the robot (in the local frame),
    the pose of the robot relative to the ship,
    the angular velocity of the robot,
    and the linear acceleration of the robot.
These classes are used to store and organize the data received from the ship and robot.
"""
class ShipState:
    def __init__(self):
        self.msg = None
        self.last_time = None
        self.omega = None 
        self.omega_dot = np.zeros(3)
        self.a_linear = np.zeros(3)

class RobotState: 
    def __init__(self):
        self.v_linear = np.zeros(3)
        self.pose_rel_ship = Pose()
        self.omega = np.zeros(3)
        self.a_linear = np.zeros(3)

class EMAFilter3D: 
    def __init__(self, cutoff_freq):
        # Calcola la costante di tempo (tau) basata sulla frequenza di taglio
        self.tau = 1.0 / (2.0 * np.pi * cutoff_freq)
        self.state = np.zeros(3)
        self.initialized = False

    def filter(self, x, dt):
        # Se è il primo valore o il dt è non valido, inizializza lo stato
        if not self.initialized or dt <= 0:
            self.state = np.copy(x)
            self.initialized = True
            return self.state
        
        # Calcola il fattore di smoothing (alpha) dinamico basato sul dt reale
        alpha = 1.0 - np.exp(-dt / self.tau)
        self.state = alpha * x + (1.0 - alpha) * self.state
        return np.copy(self.state)

#===============================================
# ROS Node 
#===============================================

"""
The ImuCompensator node is responsible for subscribing to the raw IMU data from the ship and the robot, 
as well as the robot's odometry, to compute a compensated IMU message for the robot that accounts for the ship's motion.

The node initializes by declaring parameters for enabling compensation and for the robot's spawn position.
It subscribes to the ship's raw IMU data, the robot's odometry, and the robot's raw IMU data.
When new data is received, it computes the compensation based on the ship's motion and orientation,
and publishes the compensated IMU data on the /robot/imu/compensated topic.
"""

class ImuCompensator(Node):
    def __init__(self):
        super().__init__('imu_compensator')
        self.declare_parameter('enable',True)

        # Parametri per la posizione di partenza (passati dal launch file)
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_z', 0.0)

        self.offset_x = self.get_parameter('spawn_x').value
        self.offset_y = self.get_parameter('spawn_y').value
        self.offset_z = self.get_parameter('spawn_z').value

        #Init state class
        self.ship = ShipState()
        self.robot = RobotState()

        # Inizializziamo la posizione con l'offset per evitare salti a (0,0,0)
        self.robot.pose_rel_ship.position.x = self.offset_x
        self.robot.pose_rel_ship.position.y = self.offset_y
        self.robot.pose_rel_ship.position.z = self.offset_z

        #Subscribers 
        # Subscribers sincronizzati per le IMU
        self.sub_ship_imu = message_filters.Subscriber(self, Imu, '/ship/imu/raw', qos_profile=qos_profile_sensor_data)
        self.sub_robot_imu = message_filters.Subscriber(self, Imu, '/robot/imu/raw', qos_profile=qos_profile_sensor_data)

        # Sincronizzatore: elabora i dati solo quando arrivano due messaggi vicini nel tempo (tolleranza 0.05s)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_ship_imu, self.sub_robot_imu],
            queue_size=50,
            slop=0.05
        )
        self.ts.registerCallback(self.sync_imu_cb)

        # L'odometria può rimanere asincrona, serve solo ad aggiornare il vettore posizione
        self.sub_robot_odom = self.create_subscription(Odometry, '/robot/odom', self.robot_odom_cb, qos_profile_sensor_data)
        
        #Publisher
        self.pub_imu_comp = self.create_publisher(Imu,'/robot/imu/compensated',10)

        #gravity 
        self.g_vec=np.array([0.0,0.0,9.81])

        # Filtri dinamici resistenti al jitter (EMA time-aware)
        self.omega_filter = EMAFilter3D(cutoff_freq=2.0)
        self.acc_filter = EMAFilter3D(cutoff_freq=2.0) 

        self.get_logger().info("Imu compensator initialized.")

    def robot_odom_cb(self,msg):
        # robot velocity (local frame)
        self.robot.v_linear = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])

        offset_x = self.get_parameter('spawn_x').value
        offset_y = self.get_parameter('spawn_y').value
        offset_z = self.get_parameter('spawn_z').value

        self.robot.pose_rel_ship.position.x = msg.pose.pose.position.x + offset_x
        self.robot.pose_rel_ship.position.y = msg.pose.pose.position.y + offset_y
        self.robot.pose_rel_ship.position.z = msg.pose.pose.position.z + offset_z
        self.robot.pose_rel_ship.orientation = msg.pose.pose.orientation

    def sync_imu_cb(self, ship_msg, robot_msg):
        time = robot_msg.header.stamp.sec + robot_msg.header.stamp.nanosec * 1e-9

        omega_current = np.array([
            ship_msg.angular_velocity.x,
            ship_msg.angular_velocity.y,
            ship_msg.angular_velocity.z
        ])
        
        self.ship.a_linear = np.array([
            ship_msg.linear_acceleration.x,
            ship_msg.linear_acceleration.y,
            ship_msg.linear_acceleration.z
        ])

        if self.ship.last_time is None:
            self.ship.last_time = time
            self.ship.omega = omega_current
            return

        dt = time - self.ship.last_time
        
        if dt < 0.001:
            return

        self.ship.last_time = time 

        # --- DIFESA CONTRO IL LAG ESTREMO ---
        # Se dt > 0.05s, Gazebo ha laggato. Congeliamo omega_dot per evitare picchi assurdi.
        if dt <= 0.05:
            self.ship.omega_dot = (omega_current - self.ship.omega) / dt
        
        self.ship.omega = omega_current 

        self.robot.a_linear = np.array([
            robot_msg.linear_acceleration.x, 
            robot_msg.linear_acceleration.y,
            robot_msg.linear_acceleration.z
        ])

        self.robot.omega = np.array([
            robot_msg.angular_velocity.x,
            robot_msg.angular_velocity.y,
            robot_msg.angular_velocity.z
        ])

        if self.get_parameter('enable').value:
            acc_comp, omega_comp = self.compute_compensation()
        else: 
            acc_comp, omega_comp = self.robot.a_linear, self.robot.omega

        acc_comp = self.acc_filter.filter(acc_comp, dt) 
        omega_comp = self.omega_filter.filter(omega_comp, dt)
        
        comp_msg = Imu()
        comp_msg.header = robot_msg.header 
        comp_msg.header.frame_id = "imu_link"  
        comp_msg.orientation = self.robot.pose_rel_ship.orientation
        comp_msg.orientation_covariance = robot_msg.orientation_covariance

        comp_msg.linear_acceleration.x = acc_comp[0]
        comp_msg.linear_acceleration.y = acc_comp[1]
        comp_msg.linear_acceleration.z = acc_comp[2]
        comp_msg.linear_acceleration_covariance = robot_msg.linear_acceleration_covariance

        comp_msg.angular_velocity.x = omega_comp[0]
        comp_msg.angular_velocity.y = omega_comp[1]
        comp_msg.angular_velocity.z = omega_comp[2]
        comp_msg.angular_velocity_covariance = robot_msg.angular_velocity_covariance

        self.pub_imu_comp.publish(comp_msg)        
        
    def compute_compensation(self): 
        R_r_s = R.from_quat([
            self.robot.pose_rel_ship.orientation.x,
            self.robot.pose_rel_ship.orientation.y,
            self.robot.pose_rel_ship.orientation.z, 
            self.robot.pose_rel_ship.orientation.w
        ]).as_matrix()

        R_s_r = R.from_quat([
            self.robot.pose_rel_ship.orientation.x,
            self.robot.pose_rel_ship.orientation.y,
            self.robot.pose_rel_ship.orientation.z, 
            self.robot.pose_rel_ship.orientation.w
        ]).inv().as_matrix()

        acc_tang = np.cross(
            self.ship.omega_dot,
            np.array([
                self.robot.pose_rel_ship.position.x,
                self.robot.pose_rel_ship.position.y,
                self.robot.pose_rel_ship.position.z
            ]))
        
        acc_centr = np.cross(
            self.ship.omega,
            np.cross(
                self.ship.omega,
                np.array([
                    self.robot.pose_rel_ship.position.x,
                    self.robot.pose_rel_ship.position.y,
                    self.robot.pose_rel_ship.position.z
                ])
            )
        )
        
        acc_cor = 2.0 * np.cross(
            self.ship.omega,
            R_r_s.dot(self.robot.v_linear)
        )

        acc_app = acc_tang + acc_centr + acc_cor 

        acc_comp = self.robot.a_linear - R_s_r.dot(self.ship.a_linear + acc_app) + R_s_r.dot(np.array([0.0, 0.0, 9.81]))

        omega_comp = self.robot.omega - R_s_r.dot(self.ship.omega)

        return acc_comp, omega_comp

def main(args=None):
    rclpy.init(args=args)
    imu_compensator = ImuCompensator()

    try:
        rclpy.spin(imu_compensator)
    except KeyboardInterrupt: 
        pass
    finally:
        imu_compensator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

        