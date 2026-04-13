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
from tf2_ros import Buffer,TransformListener
from scipy.spatial.transform import Rotation as R 
from geometry_msgs.msg import Pose
import numpy as np


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

"""
source: 
https://gist.github.com/moorepants/bfea1dc3d1d90bdad2b5623b4a9e9bee

David Winter filter. 
Butterworth low-pass filter of the 2nd order. 
Moorepants implmentation. 
Changed to elaborate 3D signals. 
"""

class WinterLowPass3D: 
    def __init__(self,cutoff_freq, sample_time):
        #sampling frequency
        sampling_rate = 1.0 / sample_time
        #prewarping cutoff frequency
        correction_factor = 1.0 
        corrected_cutoff_freq = np.tan(np.pi * cutoff_freq / sampling_rate) / correction_factor

        # coefficient 
        K1 = np.sqrt(2) * corrected_cutoff_freq
        K2 = corrected_cutoff_freq ** 2

        #direct coefficients
        self.a0 = K2 / (1 + K1 + K2)
        self.a1 = 2 * self.a0
        self.a2 = self.a0

        K3 = self.a1 / K2
        self.b1 = -self.a1 + K3
        self.b2 = 1.0 -self.a1 - K3

        self.x1 = np.zeros(3)
        self.x2 = np.zeros(3)
        self.y1 = np.zeros(3)
        self.y2 = np.zeros(3)

    def filter(self,x0):
        y0 = (self.a0 * x0) + (self.a1 * self.x1) + (self.a2 * self.x2) + (self.b1 * self.y1) + (self.b2 * self.y2)
        self.x2 = np.copy(self.x1)
        self.x1 = np.copy(x0)
        self.y2 = np.copy(self.y1)
        self.y1 = np.copy(y0)
        return y0


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
        self.sub_ship_imu = self.create_subscription(Imu,'/ship/imu/raw',self.ship_imu_cb,qos_profile_sensor_data)
        self.sub_robot_odom = self.create_subscription(Odometry,'/robot/odom',self.robot_odom_cb,qos_profile_sensor_data)
        self.sub_robot_imu = self.create_subscription(Imu,'/robot/imu/raw',self.robot_imu_cb,qos_profile_sensor_data)

        #Publisher
        self.pub_imu_comp = self.create_publisher(Imu,'/robot/imu/compensated',10)

        #gravity 
        self.g_vec=np.array([0.0,0.0,9.81])

        # Filter for omega to reduce noise in the angular velocity measurements of the ship, which can be noisy and affect the compensation accuracy
        self.omega_filter = WinterLowPass3D(cutoff_freq=2.0, sample_time=0.01)

        # Filter for omega_dot to reduce noise in the differentiation of angular velocity
        self.omega_dot_filter = WinterLowPass3D(cutoff_freq=1.0, sample_time=0.01)

        # Filter for linear acceleration to reduce noise in the compensation (especially for high-frequency ship motions)
        self.acc_filter = WinterLowPass3D(cutoff_freq=2.0, sample_time=0.005) 

        self.get_logger().info("Imu compensator initialized.")

    def ship_imu_cb(self,msg):
        self.ship.msg = msg 

        # current angular velocity (omega) -- read from /ship/imu/raw
        omega_current = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # save linear acceleration
        self.ship.a_linear = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # current time instant 
        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # angular acceleration -- computation from omega and last_time 
        if self.ship.last_time is not None:
            dt = time - self.ship.last_time
            if dt>0.005: # avoid too small dt that can cause noise in the differentiation
                if self.ship.omega is not None: 
                    raw_omega_dot = (omega_current - self.ship.omega) / dt
                    self.ship.omega_dot = self.omega_dot_filter.filter(raw_omega_dot)

        # Update omega and time
        self.ship.omega = omega_current
        self.ship.last_time = time 

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

    def robot_imu_cb(self,msg):
        if self.ship.msg is None or self.ship.omega is None: 
            return
        
        # save linear acceleration and angular velocity

        self.robot.a_linear = np.array([
            msg.linear_acceleration.x, 
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        self.robot.omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # Compute compensation of acceleration and omega
        if self.get_parameter('enable').value:
            acc_comp, omega_comp = self.compute_compensation()
        else: 
            acc_comp,omega_comp = self.robot.a_linear, self.robot.omega

        # filter the compensated acceleration to reduce noise
        acc_comp = self.acc_filter.filter(acc_comp) 
        # filter the compensated angular velocity to reduce noise
        omega_comp = self.omega_filter.filter(omega_comp)
        
        # create and pub the compensation message
        comp_msg = Imu()
        comp_msg.header= msg.header 
        comp_msg.header.frame_id = "imu_link"  
        comp_msg.orientation = msg.orientation
        comp_msg.orientation_covariance = msg.orientation_covariance

        comp_msg.linear_acceleration.x = acc_comp[0]
        comp_msg.linear_acceleration.y = acc_comp[1]
        comp_msg.linear_acceleration.z = acc_comp[2]
        comp_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        comp_msg.angular_velocity.x = omega_comp[0]
        comp_msg.angular_velocity.y = omega_comp[1]
        comp_msg.angular_velocity.z = omega_comp[2]
        comp_msg.angular_velocity_covariance = msg.angular_velocity_covariance

            #pub 
        self.pub_imu_comp.publish(comp_msg)

        
        
    def compute_compensation(self): 

        # Rotation matrix (to use it with Scipy method)

        #rotation matrix of ship respect to robot 
        R_r_s = R.from_quat([
            self.robot.pose_rel_ship.orientation.x,
            self.robot.pose_rel_ship.orientation.y,
            self.robot.pose_rel_ship.orientation.z, 
            self.robot.pose_rel_ship.orientation.w
        ]).as_matrix()

        #rotation matrix of robot respect to ship 
        R_s_r = R.from_quat([
            self.robot.pose_rel_ship.orientation.x,
            self.robot.pose_rel_ship.orientation.y,
            self.robot.pose_rel_ship.orientation.z, 
            self.robot.pose_rel_ship.orientation.w
        ]).inv().as_matrix()

        #==================================================================
        # computation of apparent accelerations ===========================
        #==================================================================

        # tangential acceleration
        acc_tang = np.cross(
            self.ship.omega_dot,
            np.array([
                self.robot.pose_rel_ship.position.x,
                self.robot.pose_rel_ship.position.y,
                self.robot.pose_rel_ship.position.z
            ]))
        
        # centrifugal acceleration
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
        
        # Coriolis acceleration
        acc_cor = 2.0 * np.cross(
            self.ship.omega,
            R_r_s.dot(self.robot.v_linear)
        )

        # APPARENT ACCELERATION
        acc_app = acc_tang + acc_centr + acc_cor 

        #====================================================
        #   compensation acceleration 
        #====================================================

        # acc_comp = a_raw - R_s^r * (a_ship + a_app) + R_s^r * g 
        acc_comp = self.robot.a_linear - R_s_r.dot(self.ship.a_linear + acc_app) + R_s_r.dot(self.g_vec)

        # omega_comp = omega_raw - R_s^r * omega_ship
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

        