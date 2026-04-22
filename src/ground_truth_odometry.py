#!/usr/bin/env python3

"""
Ground truth odometry node for converting TF messages to odometry messages.
Uses tf2_ros.Buffer instead of direct /tf subscription for stable transforms.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GroundTruthOdometry(Node):
    def __init__(self):
        super().__init__('ground_truth_odometry')

        self.declare_parameter('output_odom_topic', '/ground_truth/odom')
        self.declare_parameter('robot_child_frame_token', 'base_footprint')
        self.declare_parameter('robot_parent_frame_token', 'odom')
        self.declare_parameter('odom_frame_id', 'world')
        self.declare_parameter('base_frame_id', 'base_footprint_truth')
        self.declare_parameter('publish_rate', 50.0)  # Frequenza di campionamento in Hz

        output_odom_topic = self.get_parameter('output_odom_topic').get_parameter_value().string_value
        self.child_frame = self.get_parameter('robot_child_frame_token').get_parameter_value().string_value
        self.parent_frame = self.get_parameter('robot_parent_frame_token').get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.pub_odom = self.create_publisher(Odometry, output_odom_topic, qos_profile_sensor_data)

        # Inizializzazione di Buffer e Listener (gestisce internamente i conflitti e l'interpolazione)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.prev_t = None
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None

        # Timer che sostituisce la sottoscrizione diretta a /tf
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Ground truth converter enabled: Polling TF {self.parent_frame} -> {self.child_frame} '
            f'at {publish_rate} Hz. Publishing to [{output_odom_topic}]'
        )

    def timer_callback(self):
        try:
            # Richiede al buffer la trasformazione più recente e stabile disponibile
            t = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time())
        except TransformException as ex:
            # Silenzia gli errori finché l'albero TF non è completamente costruito
            self.get_logger().debug(f'Could not transform {self.parent_frame} to {self.child_frame}: {ex}')
            return

        stamp = t.header.stamp.sec + 1e-9 * t.header.stamp.nanosec
        
        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z
        q = t.transform.rotation
        yaw = _yaw_from_quaternion(q)

        vx = 0.0
        vy = 0.0
        wz = 0.0

        if self.prev_t is not None:
            dt = stamp - self.prev_t
            # Evita divisioni per zero causate da messaggi identici successivi
            if dt > 1e-6:
                vx = (x - self.prev_x) / dt
                vy = (y - self.prev_y) / dt
                wz = _wrap_angle(yaw - self.prev_yaw) / dt

        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
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