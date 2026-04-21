#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class StereoOdomTfPublisher(Node):
    def __init__(self) -> None:
        super().__init__('stereo_odom_tf_publisher')

        self.declare_parameter('odom_topic', '/stereo_odom')
        self.declare_parameter('parent_frame_id', 'stereo_odom')
        self.declare_parameter('child_frame_id', 'base_footprint_stereo')

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self._parent_frame_id = (
            self.get_parameter('parent_frame_id').get_parameter_value().string_value
        )
        self._child_frame_id = (
            self.get_parameter('child_frame_id').get_parameter_value().string_value
        )

        self._tf_broadcaster = TransformBroadcaster(self)
        self._sub = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            10,
        )

        self.get_logger().info(
            f'Publishing TF from odometry topic {odom_topic}: '
            f'{self._parent_frame_id} -> {self._child_frame_id}'
        )

    @staticmethod
    def _is_finite(*values: float) -> bool:
        return all(math.isfinite(v) for v in values)

    def _odom_callback(self, msg: Odometry) -> None:
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        if not self._is_finite(px, py, pz, qx, qy, qz, qw):
            self.get_logger().warn('Skipping TF publish: odometry pose has NaN/Inf values.')
            return

        q_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if q_norm < 1e-6:
            self.get_logger().warn('Skipping TF publish: odometry quaternion norm is too small.')
            return

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp

        if self._parent_frame_id:
            transform.header.frame_id = self._parent_frame_id
        else:
            transform.header.frame_id = msg.header.frame_id

        transform.child_frame_id = self._child_frame_id

        transform.transform.translation.x = px
        transform.transform.translation.y = py
        transform.transform.translation.z = pz
        transform.transform.rotation.x = qx / q_norm
        transform.transform.rotation.y = qy / q_norm
        transform.transform.rotation.z = qz / q_norm
        transform.transform.rotation.w = qw / q_norm

        self._tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = StereoOdomTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
