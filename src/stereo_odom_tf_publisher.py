#!/usr/bin/env python3

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

    def _odom_callback(self, msg: Odometry) -> None:
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp

        if self._parent_frame_id:
            transform.header.frame_id = self._parent_frame_id
        else:
            transform.header.frame_id = msg.header.frame_id

        transform.child_frame_id = self._child_frame_id

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

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
