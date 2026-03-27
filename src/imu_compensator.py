#!/usr/bin/env python3
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

#===============================================
# ROS Node 
#===============================================

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

        # #TF
        # self.tf_buffer = Buffer() 
        # self.tf_listener = TransformListener(self.tf_buffer,self)

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
            if dt>0.0: 
                if self.ship.omega is not None: 
                    self.ship.omega_dot = (omega_current - self.ship.omega) / dt

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

        # #TF transform from robot and ship 
        # try:
        #     # Transform: of robot respect to ship
        #     # lookup_transform(target_frame, source_frame, time)
        #     t_robot_ship = self.tf_buffer.lookup_transform( 
        #         self.ship.msg.header.frame_id,
        #         msg.header.frame_id ,
        #         rclpy.time.Time()
        #     )
        # except Exception as e:
        #     self.get_logger().warn(f"Errore TF: {e}", throttle_duration_sec=2.0)
        #     return
        
        # # Update robot position and orientation
        # self.robot.pose_rel_ship.position.x = t_robot_ship.transform.translation.x
        # self.robot.pose_rel_ship.position.y = t_robot_ship.transform.translation.y
        # self.robot.pose_rel_ship.position.z = t_robot_ship.transform.translation.z 
        # self.robot.pose_rel_ship.orientation = t_robot_ship.transform.rotation

        # Compute compensation of acceleration and omega
        if self.get_parameter('enable').value:
            acc_comp, omega_comp = self.compute_compensation()
        else: 
            acc_comp,omega_comp = self.robot.a_linear, self.robot.omega
        
        # create and pub the compensation message
        comp_msg = Imu()
        comp_msg.header= msg.header 
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()

        