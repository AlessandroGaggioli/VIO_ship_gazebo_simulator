#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import message_filters
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from rclpy.qos import qos_profile_sensor_data

class OdomComparator(Node):
    def __init__(self):
        super().__init__('odom_comparator')

        self.declare_parameter('gt_robot_topic', '/robot/ground_truth/odom')
        self.declare_parameter('ship_joints_topic', '/ship/joint_states')
        self.declare_parameter('est_topic', '')
        self.declare_parameter('sync_slop', 0.05)

        self.sub_gt_robot = message_filters.Subscriber(self, Odometry, self.get_parameter('gt_robot_topic').value, qos_profile=qos_profile_sensor_data)
        self.sub_ship_joints = message_filters.Subscriber(self, JointState, self.get_parameter('ship_joints_topic').value, qos_profile=qos_profile_sensor_data)
        self.sub_est = message_filters.Subscriber(self, Odometry, self.get_parameter('est_topic').value, qos_profile=qos_profile_sensor_data)
      
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_gt_robot, self.sub_ship_joints, self.sub_est], queue_size=2000, slop=self.get_parameter('sync_slop').value)
        self.ts.registerCallback(self.sync_callback)

        self.start_time = None
        self.joint_idx = None
        self.init_gt_pos, self.init_est_pos = None, None
        self.init_gt_rot, self.init_est_rot = None, None
        self.history = {'t': [], 'gt': [], 'est': [], 'e_pos': [], 'e_ori': [], 'e_lin': [], 'e_ang': []}

    def _vec(self, msg): return np.array([msg.x, msg.y, msg.z])
    def _quat(self, msg): return np.array([msg.x, msg.y, msg.z, msg.w])
    def wrap_angle(self, angle): return (angle + np.pi) % (2 * np.pi) - np.pi

    def sync_callback(self, msg_gt_robot, msg_ship_joints, msg_est):
        curr_t = msg_est.header.stamp.sec + msg_est.header.stamp.nanosec * 1e-9

        if self.joint_idx is None:
            names = msg_ship_joints.name
            self.joint_idx = [names.index(j) if j in names else -1 for j in ['heave_joint', 'roll_joint', 'pitch_joint']]

        p, v = msg_ship_joints.position, msg_ship_joints.velocity
        h_p, r_p, pi_p = [p[i] if i != -1 else 0.0 for i in self.joint_idx]
        h_v, r_v, pi_v = [v[i] if i != -1 else 0.0 for i in self.joint_idx]

        pos_robot_w = self._vec(msg_gt_robot.pose.pose.position)
        rot_robot_w = R.from_quat(self._quat(msg_gt_robot.pose.pose.orientation))
        
        ship_pos_w = np.array([0.0, 0.0, h_p])
        ship_rot_w = R.from_euler('XY', [r_p, pi_p])

        gt_pos = ship_rot_w.inv().apply(pos_robot_w - ship_pos_w)
        gt_rot = ship_rot_w.inv() * rot_robot_w

        v_ship_w = np.array([0.0, 0.0, h_v]) + np.cross([r_v, pi_v, 0.0], pos_robot_w - ship_pos_w)
        inv_rot = rot_robot_w.inv()
        gt_v_lin = self._vec(msg_gt_robot.twist.twist.linear) - inv_rot.apply(v_ship_w)
        gt_v_ang = self._vec(msg_gt_robot.twist.twist.angular) - inv_rot.apply([r_v, pi_v, 0.0])

        est_pos = self._vec(msg_est.pose.pose.position)
        est_rot = R.from_quat(self._quat(msg_est.pose.pose.orientation))
        
        if self.start_time is None:
            self.start_time = curr_t
            self.init_gt_pos, self.init_est_pos = gt_pos, est_pos
            self.init_gt_rot, self.init_est_rot = gt_rot, est_rot

        rel_gt_pos = gt_pos - self.init_gt_pos
        rel_est_pos = est_pos - self.init_est_pos
        euler_gt = (self.init_gt_rot.inv() * gt_rot).as_euler('xyz')
        euler_est = (self.init_est_rot.inv() * est_rot).as_euler('xyz')
        
        self.history['t'].append(curr_t - self.start_time)
        self.history['gt'].append(rel_gt_pos)
        self.history['est'].append(rel_est_pos)
        self.history['e_pos'].append(rel_est_pos - rel_gt_pos)
        self.history['e_ori'].append(self.wrap_angle(euler_est - euler_gt))
        self.history['e_lin'].append(self._vec(msg_est.twist.twist.linear) - gt_v_lin)
        self.history['e_ang'].append(self._vec(msg_est.twist.twist.angular) - gt_v_ang)

    def plot_results(self):
        if not self.history['t']: return
            
        h = {k: np.array(v) for k, v in self.history.items()}
        ates = {k: np.linalg.norm(h[k], axis=1) for k in ['e_pos', 'e_ori', 'e_lin', 'e_ang']}

        fig, axs = plt.subplots(3, 2, figsize=(15, 10))
        axs[0,0].plot(h['gt'][:,0], h['gt'][:,1], 'k--', label='GT locale')
        axs[0,0].plot(h['est'][:,0], h['est'][:,1], 'b', label='Stima')
        axs[0,0].set_title('Trajectory 2D'); axs[0,0].legend(); axs[0,0].axis('equal'); axs[0,0].grid(True)
        
        for i, (k, title, color) in enumerate(zip(ates.keys(), ['Position [m]', 'Orientation [rad]', 'Linear Vel. [m/s]', 'Angular Vel. [rad/s]'], ['red', 'orange', 'green', 'purple'])):
            ax = axs[(i+1)//2, (i+1)%2]
            ax.plot(h['t'], ates[k], color=color)
            ax.set_title(f'ATE {title}'); ax.grid(True)
            
        plt.tight_layout(); plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = OdomComparator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.plot_results()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()