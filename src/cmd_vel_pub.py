#!/usr/bin/env python3

"""
ROS2 cmd_vel Publisher with keyboard-triggered trajectory (single node, no threads)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import yaml
import os
import sys
import termios
import tty
import select
from ament_index_python.packages import get_package_share_directory


def get_key():
    """Non-blocking keyboard read"""
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None


class TrajectorySegment:
    def __init__(self,
                 linear_x,
                 linear_y,
                 linear_z,
                 angular_x,
                 angular_y,
                 angular_z,
                 duration,
                 name=""):

        self.linear_x = linear_x
        self.linear_y = linear_y
        self.linear_z = linear_z
        self.angular_x = angular_x
        self.angular_y = angular_y
        self.angular_z = angular_z
        self.duration = duration
        self.name = name


class CmdVelTrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('cmd_vel_pub')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter('publish_rate', 10.0)
        self.publish_rate = self.get_parameter('publish_rate').value

        self.declare_parameter('output_file', 'trajectory.yaml')
        self.config_file = self.get_parameter('output_file').value

        # trajectory state
        self.is_executing = False
        self.current_segment_idx = 0
        self.segment_start_time = None
        self.trajectory = []
        self.total_duration = 0.0

        # setup keyboard (non-blocking)
        self.old_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.load_traj(self.config_file)

        if not self.trajectory:
            raise RuntimeError("Failed to load trajectory")

        self.get_logger().info("=== cmd_vel trajectory ===")
        self.get_logger().info(f"Loaded {len(self.trajectory)} segments")
        self.get_logger().info(f"Total duration: {self.total_duration:.2f} seconds")
        self.get_logger().info("Press ENTER to start trajectory")

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def load_traj(self, config_file):
        try:
            if os.path.isabs(config_file):
                config_path = config_file
            else:
                pkg_share = get_package_share_directory('ship_gazebo')
                ws_root = os.path.abspath(os.path.join(pkg_share, '../../../../'))
                config_path = os.path.join(ws_root, 'src', 'ship_gazebo', 'config', config_file)

            if not os.path.exists(config_path):
                self.get_logger().error(f"Config file not found: {config_path}")
                return

            self.get_logger().info(f"Loading trajectory from: {config_path}")

            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)

            if not data or 'trajectory' not in data:
                self.get_logger().error("Invalid YAML format")
                return

            self.trajectory = []

            for i, segment_data in enumerate(data['trajectory']):
                try:
                    seg = TrajectorySegment(
                        linear_x=float(segment_data.get('linear_x', 0.0)),
                        linear_y=float(segment_data.get('linear_y', 0.0)),
                        linear_z=float(segment_data.get('linear_z', 0.0)),
                        angular_x=float(segment_data.get('angular_x', 0.0)),
                        angular_y=float(segment_data.get('angular_y', 0.0)),
                        angular_z=float(segment_data.get('angular_z', 0.0)),
                        duration=float(segment_data.get('duration', 1.0)),
                        name=segment_data.get('name', f"Segment {i+1}")
                    )
                    self.trajectory.append(seg)

                except Exception as e:
                    self.get_logger().error(f"Error parsing segment {i}: {e}")

            self.total_duration = sum(seg.duration for seg in self.trajectory)

        except Exception as e:
            self.get_logger().error(f"Error loading config: {e}")

    def start_trajectory(self):
        self.is_executing = True
        self.current_segment_idx = 0
        self.segment_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info("Trajectory started!")

    def timer_callback(self):
        if not rclpy.ok():
            return

        # keyboard handling
        key = get_key()
        if key == '\n':
            if not self.is_executing:
                self.start_trajectory()
            else:
                self.get_logger().warn("Already executing!")

        # if not executing → publish zero
        if not self.is_executing:
            self.publisher.publish(Twist())
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.segment_start_time

        if elapsed >= self.total_duration:
            self.get_logger().info("Trajectory completed")
            self.is_executing = False
            self.publisher.publish(Twist())
            return

        # find current segment
        time_acc = 0.0
        for idx, segment in enumerate(self.trajectory):
            if time_acc + segment.duration > elapsed:
                self.current_segment_idx = idx
                segment_elapsed = elapsed - time_acc
                break
            time_acc += segment.duration

        segment = self.trajectory[self.current_segment_idx]

        msg = Twist()
        msg.linear.x = segment.linear_x
        msg.linear.y = segment.linear_y
        msg.linear.z = segment.linear_z
        msg.angular.x = segment.angular_x
        msg.angular.y = segment.angular_y
        msg.angular.z = segment.angular_z

        self.publisher.publish(msg)

        # log once per segment start
        if segment_elapsed < (1.0 / self.publish_rate + 0.01):
            self.get_logger().info(
                f"[{self.current_segment_idx+1}/{len(self.trajectory)}] "
                f"{segment.name} | lin_x={segment.linear_x:.2f}, ang_z={segment.angular_z:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelTrajectoryExecutor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[cmd_vel_pub]: Shutting down\n")

    finally:
        # restore terminal
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.old_terminal_settings)
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()