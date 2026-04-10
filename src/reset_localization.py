#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class ResetLocalizationClient(Node):
    def __init__(self):
        super().__init__('reset_localization_client')
        self.cli = self.create_client(Empty, '/rtabmap/reset')
        self.req = Empty.Request()
        self.service_call_done = False

        # Timer to delay the service call
        self.declare_parameter('wait_time', 3.0)
        self.timer = self.create_timer(self.get_parameter('wait_time').value, self.try_reset_localization)

    def try_reset_localization(self):
        self.timer.cancel()  # Cancel the timer, we only need to call this once

        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '/rtabmap/reset service not available after 5s timeout!'
            )
            rclpy.shutdown()
            return

        future = self.cli.call_async(self.req)
        future.add_done_callback(self.service_callback)
    
    def service_callback(self, future):
        try:
            response = future.result()
            self.service_call_done = True
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ResetLocalizationClient()

    rclpy.spin(node)

if __name__ == '__main__':
    main()