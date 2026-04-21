#!/usr/bin/env python3

"""
Ground truth odometry node for converting TF messages to odometry messages.

This node listens to TF messages (e.g., from Gazebo world state) and extracts the robot's pose 
based on specified child and parent frame tokens. 
It then computes the robot's velocity and publishes it as an Odometry message.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


def _match_robot_transform(msg: TFMessage, child_token: str, parent_token: str):
    child_token = child_token.strip()
    parent_token = parent_token.strip()

    if not child_token:
        return None

    exact = []
    partial = []
    child_only = []
    heuristic = []

    for tr in msg.transforms:
        child = tr.child_frame_id
        parent = tr.header.frame_id

        parent_ok = True
        if parent_token:
            parent_ok = (parent == parent_token) or (parent_token in parent)

        if not parent_ok:
            continue

        if child == child_token:
            exact.append(tr)
        elif child_token in child:
            partial.append(tr)

        if child_token in child:
            child_only.append(tr)

        if ('base_footprint' in child) and (('odom' in parent) or parent == ''):
            heuristic.append(tr)

    if exact:
        return exact[0]
    if partial:
        return partial[0]
    if child_only:
        return child_only[0]
    if heuristic:
        return heuristic[0]
    return None


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GroundTruthOdometry(Node):
    def __init__(self):
        super().__init__('ground_truth_odometry')

        self.declare_parameter('input_tf_topic', '/tf')
        self.declare_parameter('output_odom_topic', '/ground_truth/odom')
        self.declare_parameter('robot_child_frame_token', 'base_footprint')
        self.declare_parameter('robot_parent_frame_token', 'odom')
        self.declare_parameter('odom_frame_id', 'world')
        self.declare_parameter('base_frame_id', 'base_footprint_truth')

        input_tf_topic = self.get_parameter('input_tf_topic').get_parameter_value().string_value
        output_odom_topic = self.get_parameter('output_odom_topic').get_parameter_value().string_value
        self.robot_child_frame_token = self.get_parameter('robot_child_frame_token').get_parameter_value().string_value
        self.robot_parent_frame_token = self.get_parameter('robot_parent_frame_token').get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value

        self.pub_odom = self.create_publisher(Odometry, output_odom_topic, qos_profile_sensor_data)
        self.sub_tf = self.create_subscription(
            TFMessage,
            input_tf_topic,
            self.tf_callback,
            qos_profile_sensor_data,
        )

        self.prev_t = None
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        self.no_match_counter = 0

        self.get_logger().info(
            f'Ground truth converter enabled: TF [{input_tf_topic}] -> Odom [{output_odom_topic}], '
            f'child token="{self.robot_child_frame_token}", '
            f'parent token="{self.robot_parent_frame_token}"'
        )

    def tf_callback(self, msg: TFMessage):
        tr = _match_robot_transform(msg, self.robot_child_frame_token, self.robot_parent_frame_token)
        if tr is None:
            self.no_match_counter += 1
            if self.no_match_counter % 100 == 0:
                pairs = []
                for sample in msg.transforms[:6]:
                    pairs.append(f'{sample.header.frame_id}->{sample.child_frame_id}')
                self.get_logger().warn(
                    'No matching transform in /tf yet. '
                    f'child token="{self.robot_child_frame_token}", '
                    f'parent token="{self.robot_parent_frame_token}", '
                    f'sample pairs={pairs}'
                )
            return

        stamp = tr.header.stamp.sec + 1e-9 * tr.header.stamp.nanosec
        x = tr.transform.translation.x
        y = tr.transform.translation.y
        q = tr.transform.rotation
        yaw = _yaw_from_quaternion(q)

        vx = 0.0
        vy = 0.0
        wz = 0.0

        if self.prev_t is not None:
            dt = stamp - self.prev_t
            if dt > 1e-6:
                vx = (x - self.prev_x) / dt
                vy = (y - self.prev_y) / dt
                wz = _wrap_angle(yaw - self.prev_yaw) / dt

        odom = Odometry()
        odom.header.stamp = tr.header.stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = tr.transform.translation.z
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.pub_odom.publish(odom)

        self.prev_t = stamp
        self.prev_x = x
        self.prev_y = y
        self.prev_yaw = yaw


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
