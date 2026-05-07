#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import RGBDImage
from sensor_msgs.msg import Image


class RGBDImageRelay(Node):

    def __init__(self):
        super().__init__('rgbd_image_relay')
        self.pub = self.create_publisher(Image, '/stereo_camera/image_rgb', qos_profile_sensor_data)
        self.create_subscription(RGBDImage, '/stereo_camera/rgbd_image', self._cb, qos_profile_sensor_data)

    def _cb(self, msg):
        self.pub.publish(msg.rgb)


def main(args=None):
    rclpy.init(args=args)
    node = RGBDImageRelay()
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
