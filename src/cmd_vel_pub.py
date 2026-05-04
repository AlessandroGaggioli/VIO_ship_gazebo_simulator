#!/usr/bin/env python3

""" 
ROS2 cmd_vel Publisher with ROS Clock Synchronization
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import yaml
import os
import threading
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

class TrajectorySegment: 

    def __init__(self,
        linear_x, 
        linear_y, 
        linear_z, 
        angular_x, 
        angular_y, 
        angular_z, 
        duration,
        name = ""):
            
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

        self.publisher = self.create_publisher(Twist,'/cmd_vel',10)

        self.declare_parameter('publish_rate',10.0)
        self.publish_rate = self.get_parameter('publish_rate').value

        self.declare_parameter('config_file', 'trajectory.yaml')
        self.config_file = self.get_parameter('config_file').value

        self.is_executing = False 
        self.current_segment_idx = 0 
        self.segment_start_time = None 
        self.trajectory = []
        self.total_duration = 0.0

        self.load_traj(self.config_file)

        if not self.trajectory: 
                raise RuntimeError("Failed to load trajectory\n")
        
        self.get_logger().info("=== cmd_vel trajectory ===")
        self.get_logger().info(f"Loaded {len(self.trajectory)} trajectory segments")
        self.get_logger().info(f"Total duration: {self.total_duration:.2f} seconds")
        self.get_logger().info("Press ENTER to start trajectory")

        self.timer = self.create_timer(1.0 / self.publish_rate,self.timer_callback)

        self.input_thread = threading.Thread(target = self.input_listener,daemon=True)
        self.input_thread.start()

    def load_traj(self,config_file):

        try: 
            pkg_share = get_package_share_directory('ship_gazebo')
            ws_root = os.path.abspath(os.path.join(pkg_share, '../../../../'))
            config_path = os.path.join(ws_root, 'src', 'ship_gazebo', 'config', config_file)

            if not os.path.exists(config_path):
                self.get_logger().error(f"Config file not found: {config_path}")
                return
            
            else: 
                self.get_logger().info(f"Loading trajectory from: {config_path}")

        except Exception as e:
            self.get_logger().error(f"Error loading config file: {e}")


        try:
            if not config_path.endswith(('.yaml', '.yml')):
                self.get_logger().error(f"Unsupported file format: {config_path}")
                return

            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)

            if not data:
                self.get_logger().error("Empty or invalid YAML file")
                return
            
            if 'trajectory' not in data:
                self.get_logger().error("'trajectory' key not found in config file")
                return

            self.trajectory = []

            for i,segment_data in enumerate(data['trajectory']):
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

                except (ValueError, TypeError) as e:
                    self.get_logger().error(f"Error parsing segment {i}: {e}")
                    continue
            
            self.total_duration = sum(seg.duration for seg in self.trajectory)
            self.get_logger().info(f"Successfully loaded {len(self.trajectory)} segments")

        except Exception as e: 
            self.get_logger().error(f"Error loading config file: {e}")


    def input_listener(self):
        """Listen for user input to start trajectory execution"""
        while rclpy.ok():
            try:
                input()
                if not self.is_executing:
                    self.start_trajectory()
                else:
                    self.get_logger().warn("Trajectory already executing!")
            except (EOFError, KeyboardInterrupt):
                break

    def start_trajectory(self):
        """Start executing the trajectory"""
        self.is_executing = True
        self.current_segment_idx = 0
        self.segment_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info("Trajectory execution started!")

    def timer_callback(self):
        
        if not self.is_executing: 
            msg=Twist()
            self.publisher.publish(msg)
            return
        
        current_time = self.get_clock().now().nanoseconds/1e9
        elapsed = current_time - self.segment_start_time

        if elapsed >= self.total_duration: 
            self.get_logger().info("Trajectory completed")
            self.is_executing = False 

            msg=Twist()
            self.publisher.publish(msg)
            return 
        
        time = 0.0 

        for idx, segment in enumerate(self.trajectory):
            if time + segment.duration > elapsed:
                self.current_segment_idx = idx
                segment_elapsed = elapsed - time
                break
            time += segment.duration


        segment = self.trajectory[self.current_segment_idx]
        msg = Twist()
        msg.linear.x = segment.linear_x
        msg.linear.y = segment.linear_y
        msg.linear.z = segment.linear_z
        msg.angular.x = segment.angular_x
        msg.angular.y = segment.angular_y
        msg.angular.z = segment.angular_z

        self.publisher.publish(msg)

        if segment_elapsed < (1.0 / self.publish_rate+0.01): 
            self.get_logger().info(
                f"[{self.current_segment_idx + 1}/{len(self.trajectory)}] {segment} "
                f"| lin_x={segment.linear_x:.2f} m/s, "
                f"ang_z={segment.angular_z:.2f} rad/s"
            )

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelTrajectoryExecutor()

    try:
        rclpy.spin(node)
    except:
        node.get_logger().info("Shutting down")

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
            
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()