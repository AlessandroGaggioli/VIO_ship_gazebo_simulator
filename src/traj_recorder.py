#!/usr/bin/env python3 

"""
This node subscribes to /cmd_vel topic to record the Twist meessages and 
save <traj.yaml> file to reconstruct the behaviour for other test. 
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import yaml
from ament_index_python.packages import get_package_share_directory
import os

class TrajRecorder(Node): 
    def __init__(self):
        super().__init__('traj_recorder')
        self.subscription=self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

        self.traj = []
        self.current_twist = None 
        self.segment_start_time = None 
        self.is_started = False 
        self.segment_count = 1 

        pkg_share = get_package_share_directory('ship_gazebo')
        ws_root = os.path.abspath(os.path.join(pkg_share, '../../../../'))
        src_config_dir = os.path.join(ws_root, 'src', 'ship_gazebo', 'config')
        
        if not os.path.exists(src_config_dir):
            os.makedirs(src_config_dir)

        self.declare_parameter('config_file', 'trajectory.yaml')
        self.filename = os.path.join(src_config_dir, self.get_parameter('config_file').value)

        self.get_logger().info("Wait for moving the robot\n")

    def is_zero_twist(self,msg):
        return (msg.linear.x == 0.0 and 
                msg.linear.y == 0.0 and 
                msg.linear.z == 0.0 and
                msg.angular.x == 0.0 and 
                msg.angular.y == 0.0 and 
                msg.angular.z == 0.0)
    
    def is_twist_diff(self,t1,t2,tolerance=1e-4):
        if t1 is None or t2 is None:
            return True
        return (abs(t1.linear.x - t2.linear.x) > tolerance or
                abs(t1.linear.y - t2.linear.y) > tolerance or
                abs(t1.angular.z - t2.angular.z) > tolerance)
    
    def listener_callback(self,msg):
        now = self.get_clock().now()

        if not self.is_started:
            if self.is_zero_twist(msg):
                return 
            
            self.is_started = True 
            self.current_twist = msg 
            self.segment_start_time = now
            self.get_logger().info("Starting recording\n")
            return 
        
        if self.is_twist_diff(self.current_twist,msg):
            duration = (now.nanoseconds - self.segment_start_time.nanoseconds) / 1e9

            if duration > 0.05 : 
                self.save_segment(duration)

            self.current_twist = msg 
            self.segment_start_time = now
        
    def save_segment(self, duration, is_shutting_down=False):
        self.traj.append({
            'name':f"Segment {self.segment_count}",
            'linear_x': float(self.current_twist.linear.x),
            'linear_y': float(self.current_twist.linear.y),
            'linear_z': float(self.current_twist.linear.z),
            'angular_x': float(self.current_twist.angular.x),
            'angular_y': float(self.current_twist.angular.y),
            'angular_z': float(self.current_twist.angular.z),
            'duration': round(duration, 3)
        })
        
        msg = f"Registered segment {self.traj[-1]}"
        if is_shutting_down:
            print(f"[traj_recorder]: {msg}")
        else:
            self.get_logger().info(msg)
            
        self.segment_count += 1

    def save_yaml(self):
        print("\n[traj_recorder]: saving trajecory...")
        
        if self.is_started and self.current_twist is not None:
            try:
                now = self.get_clock().now()
                duration = (now.nanoseconds - self.segment_start_time.nanoseconds) / 1e9
                if duration > 0.05:
                    self.save_segment(duration, is_shutting_down=True)
            except Exception:
                pass 

        if not self.traj:
            print("[traj_recorder]: No motion recorded . File YAML not created.")
            return 
        
        output = {'trajectory':self.traj}

        try: 
            with open(self.filename, 'w') as f:
                yaml.dump(output, f, sort_keys=False, default_flow_style=False)
            
            print(f"[traj_recorder]: SUCCESS - File yaml saved in:")
            print(f"[traj_recorder]: {self.filename}\n")
        
        except Exception as e: 
            print(f"[traj_recorder]: ERROR - Error saving file: {e}\n")

def main(args=None):
    rclpy.init(args=args)
    node = TrajRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[traj_recorder]: Ctrl-C --- closing...")
        node.save_yaml()
    finally: 
        try:
            node.destroy_node()
        except Exception:
            pass
            
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    main()