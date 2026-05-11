#!/usr/bin/env python3

"""
Node to compare raw and compensated IMU data from the robot with the ship's IMU data, and log them to a CSV file for offline analysis.
The script subscribes to three IMU topics:
- /robot/imu/raw: the raw IMU data from the robot (before compensation)
- /robot/imu/compensated: the compensated IMU data from the robot 
- /ship/imu/raw: the raw IMU data from the ship 
The script performs approximate-time synchronization of the three streams, ensuring that only samples that are close in time are logged together.
The synchronized data is written to a CSV file with columns for timestamps, raw robot IMU, compensated robot IMU, and ship IMU values.
Skips the first 10 seconds of data collection.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
import csv
import matplotlib.pyplot as plt
from collections import deque
from scipy.spatial.transform import Rotation
from pathlib import Path
import time

class ImuLogger(Node):
    def __init__(self):
        super().__init__('imu_comparator')
        
        # Declare ROS2 parameters
        self.declare_parameter('csv_output_path', '')
        self.declare_parameter('pdf_output_path', '')
        
        self.csv_output_path = self.get_parameter('csv_output_path').value
        self.pdf_output_path = self.get_parameter('pdf_output_path').value
        
        # Only open file if csv_output_path is specified
        self.file = None
        self.writer = None
        if self.csv_output_path:
            output_file = Path(self.csv_output_path)
            output_file.parent.mkdir(parents=True, exist_ok=True)
            try:
                self.file = open(self.csv_output_path, 'w', newline='')
                self.writer = csv.writer(self.file)
                self.get_logger().info(f"CSV file opened for writing: {self.csv_output_path}")
                self.writer.writerow([
                'timestamp',
                # Raw robot IMU
                'raw_acc_x', 'raw_acc_y', 'raw_acc_z',
                'raw_omega_x', 'raw_omega_y', 'raw_omega_z',
                'raw_roll', 'raw_pitch', 'raw_yaw',
                # Compensated robot IMU
                'comp_acc_x', 'comp_acc_y', 'comp_acc_z',
                'comp_omega_x', 'comp_omega_y', 'comp_omega_z',
                'comp_roll', 'comp_pitch', 'comp_yaw',
                # Ship IMU
                'ship_acc_x', 'ship_acc_y', 'ship_acc_z',
                'ship_omega_x', 'ship_omega_y', 'ship_omega_z',
                'ship_roll', 'ship_pitch', 'ship_yaw'
            ])
            except Exception as e:
                self.get_logger().error(f"Failed to open CSV file: {e}")
                self.file = None
                self.writer = None
        
        # Subscribe to three IMU streams
        self.sub_raw = self.create_subscription(Imu, '/robot/imu/raw', self.raw_cb, qos_profile_sensor_data)
        self.sub_comp = self.create_subscription(Imu, '/robot/imu/compensated', self.comp_cb, qos_profile_sensor_data)
        self.sub_raw_ship = self.create_subscription(Imu, '/ship/imu/raw', self.ship_imu_cb, qos_profile_sensor_data) 
        
        # Buffers for plotting time-series of each axis
        self.t_data = []
        self.raw_acc = {'x': [], 'y': [], 'z': []}
        self.raw_omega = {'x': [], 'y': [], 'z': []}
        self.comp_acc = {'x': [], 'y': [], 'z': []}
        self.comp_omega = {'x': [], 'y': [], 'z': []}
        self.ship_acc = {'x': [], 'y': [], 'z': []}
        self.ship_omega = {'x': [], 'y': [], 'z': []}
        self.raw_orientation = {'x': [], 'y': [], 'z': []}
        self.comp_orientation = {'x': [], 'y': [], 'z': []}
        self.ship_orientation = {'x': [], 'y': [], 'z': []}

        # Approximate-time synchronization settings
        self.max_sync_dt = 0.02  # 20 ms
        self.max_history_sec = 0.5
        self.raw_buffer = deque(maxlen=5000)
        self.ship_buffer = deque(maxlen=5000)
        self.start_time = None
        self.skip_duration = 10.0  # Skip first 10 seconds
        
        self.sample_count = 0

        save_msg = f" Writing to '{self.csv_output_path}'." if self.csv_output_path else " (CSV saving disabled)"
        self.get_logger().info(f"IMU Logger initialized.{save_msg} Skipping first 10 seconds. Waiting for data... Press Ctrl+C to stop.")

    @staticmethod
    def msg_time(msg):
        # Convert ROS2 header stamp to float seconds.
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    @staticmethod
    def find_closest(buffer, target_t):
        # Find the message whose timestamp is nearest to target_t.
        if not buffer:
            return None, None
        best_t, best_msg = min(buffer, key=lambda pair: abs(pair[0] - target_t))
        return best_t, best_msg

    @staticmethod
    def prune_old(buffer, cutoff_t):
        # Drop old samples to keep lookup efficient and memory bounded.
        while buffer and buffer[0][0] < cutoff_t:
            buffer.popleft()

    def raw_cb(self, msg):
        # Store timestamped raw robot IMU sample.
        self.raw_buffer.append((self.msg_time(msg), msg))

    def ship_imu_cb(self, msg):
        # Store timestamped raw ship IMU sample.
        self.ship_buffer.append((self.msg_time(msg), msg))

    def comp_cb(self, msg):
        # Use compensated timestamp as reference and align the other streams to it.
        current_time = self.msg_time(msg)

        # Remove stale samples before matching.
        cutoff_t = current_time - self.max_history_sec
        self.prune_old(self.raw_buffer, cutoff_t)
        self.prune_old(self.ship_buffer, cutoff_t)

        # Nearest-neighbor matching for raw and ship IMU.
        raw_t, raw_msg = self.find_closest(self.raw_buffer, current_time)
        ship_t, ship_msg = self.find_closest(self.ship_buffer, current_time)

        # Skip this cycle if any stream is missing.
        if raw_msg is None or ship_msg is None:
            return

        # Accept only synchronized triplets within tolerance.
        if abs(raw_t - current_time) > self.max_sync_dt or abs(ship_t - current_time) > self.max_sync_dt:
            return

        # Build a relative timeline that starts from the first valid sample.
        if self.start_time is None:
            self.start_time = current_time
        t = current_time - self.start_time

        # --- Raw data ---
        ra_x = raw_msg.linear_acceleration.x
        ra_y = raw_msg.linear_acceleration.y
        ra_z = raw_msg.linear_acceleration.z
        ro_x = raw_msg.angular_velocity.x
        ro_y = raw_msg.angular_velocity.y
        ro_z = raw_msg.angular_velocity.z
        rq = [
            raw_msg.orientation.x,
            raw_msg.orientation.y,
            raw_msg.orientation.z,
            raw_msg.orientation.w 
        ]

        # --- compensated data ---
        ca_x = msg.linear_acceleration.x
        ca_y = msg.linear_acceleration.y
        ca_z = msg.linear_acceleration.z
        co_x = msg.angular_velocity.x
        co_y = msg.angular_velocity.y
        co_z = msg.angular_velocity.z
        cq = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w 
        ]

        # --- ship data ---
        sa_x = ship_msg.linear_acceleration.x
        sa_y = ship_msg.linear_acceleration.y
        sa_z = ship_msg.linear_acceleration.z
        so_x = ship_msg.angular_velocity.x
        so_y = ship_msg.angular_velocity.y
        so_z = ship_msg.angular_velocity.z
        sq = [
            ship_msg.orientation.x,
            ship_msg.orientation.y,
            ship_msg.orientation.z,
            ship_msg.orientation.w 
        ]

        # --- Quaternions to Roll Pitch Yaw conversion ---
        rr = Rotation.from_quat(rq).as_euler('xyz', degrees=False)
        rr_x, rr_y, rr_z = rr

        cr = Rotation.from_quat(cq).as_euler('xyz', degrees=False)
        cr_x, cr_y, cr_z = cr
        
        sr = Rotation.from_quat(sq).as_euler('xyz', degrees=False)
        sr_x, sr_y, sr_z = sr

        # --- Skip first 10 seconds of data: do not record, but continue listening ---
        if t < self.skip_duration:
            return

        # --- Write synchronized values to CSV (if enabled) ---
        if self.writer:
            self.writer.writerow([
                t - self.skip_duration,  # Write relative time (starts from 0 after skip)
                ra_x, ra_y, ra_z, ro_x, ro_y, ro_z, rr_x, rr_y, rr_z,
                ca_x, ca_y, ca_z, co_x, co_y, co_z, cr_x, cr_y, cr_z,
                sa_x, sa_y, sa_z, so_x, so_y, so_z, sr_x, sr_y, sr_z
            ])
            self.sample_count += 1

            # Flush periodically so data is not kept in memory for too long.
            if self.sample_count % 100 == 0:
                self.file.flush()

        # Append same synchronized values to plotting arrays.
        self.t_data.append(t - self.skip_duration)  # Adjust time to start from 0 after skip
        
        self.raw_acc['x'].append(ra_x)
        self.raw_acc['y'].append(ra_y)
        self.raw_acc['z'].append(ra_z)
        self.raw_omega['x'].append(ro_x)
        self.raw_omega['y'].append(ro_y)
        self.raw_omega['z'].append(ro_z)
        self.raw_orientation['x'].append(rr_x)
        self.raw_orientation['y'].append(rr_y)
        self.raw_orientation['z'].append(rr_z)

        self.comp_acc['x'].append(ca_x)
        self.comp_acc['y'].append(ca_y)
        self.comp_acc['z'].append(ca_z)
        self.comp_omega['x'].append(co_x)
        self.comp_omega['y'].append(co_y)
        self.comp_omega['z'].append(co_z)
        self.comp_orientation['x'].append(cr_x)
        self.comp_orientation['y'].append(cr_y)
        self.comp_orientation['z'].append(cr_z)

        self.ship_acc['x'].append(sa_x)
        self.ship_acc['y'].append(sa_y)
        self.ship_acc['z'].append(sa_z)
        self.ship_omega['x'].append(so_x)
        self.ship_omega['y'].append(so_y)
        self.ship_omega['z'].append(so_z)
        self.ship_orientation['x'].append(sr_x)
        self.ship_orientation['y'].append(sr_y)
        self.ship_orientation['z'].append(sr_z)


def plot_results(node):
    """Plot only if at least one synchronized sample has been recorded."""
    if not node.t_data:
        print("No data collected to plot.")
        return

    # Layout: first row acceleration (x,y,z), second row angular velocity (x,y,z), third row orientation (x,y,z).
    fig, axs = plt.subplots(3, 3, figsize=(16, 9))
    fig.suptitle('Comparison IMU Robot: Raw (Red) vs Compensated (Blue) + Ship (Green)', fontsize=16)

    asse_nomi = ['x', 'y', 'z']
    titoli_acc = ['Linear Acceleration X', 'Linear Acceleration Y', 'Linear Acceleration Z']
    titoli_omega = ['Angular Velocity X', 'Angular Velocity Y', 'Angular Velocity Z']
    titoli_orientation = ['Orientation R', 'Orientation P', 'Orientation Y']

    # First row: linear acceleration comparison for x, y, z.
    for i, asse in enumerate(asse_nomi):
        axs[0, i].plot(node.t_data, node.raw_acc[asse], label='Raw', color='red', alpha=0.6, linewidth=1.5)
        axs[0, i].plot(node.t_data, node.comp_acc[asse], label='Compensated', color='blue', alpha=0.8, linewidth=1.5)
        axs[0, i].plot(node.t_data, node.ship_acc[asse], label='Ship', color='green', alpha=0.8, linewidth=1.5)
        axs[0, i].set_title(titoli_acc[i])
        axs[0, i].set_xlabel('time (s)')
        axs[0, i].set_ylabel('m/s^2')
        axs[0, i].grid(True, linestyle='--', alpha=0.7)

    # Second row: angular velocity comparison for x, y, z.
    for i, asse in enumerate(asse_nomi):
        axs[1, i].plot(node.t_data, node.raw_omega[asse], label='Raw', color='red', alpha=0.6, linewidth=1.5)
        axs[1, i].plot(node.t_data, node.comp_omega[asse], label='Compensated', color='blue', alpha=0.8, linewidth=1.5)
        axs[1, i].plot(node.t_data, node.ship_omega[asse], label='Ship', color='green', alpha=0.8, linewidth=1.5)
        axs[1, i].set_title(titoli_omega[i])
        axs[1, i].set_xlabel('time (s)')
        axs[1, i].set_ylabel('rad/s')
        axs[1, i].grid(True, linestyle='--', alpha=0.7)

    # Third row: orientation comparison for roll,pitch,yaw (x,y,z)
    for i, asse in enumerate(asse_nomi):
        axs[2, i].plot(node.t_data, node.raw_orientation[asse], label='Raw', color='red', alpha=0.6, linewidth=1.5)
        axs[2, i].plot(node.t_data, node.comp_orientation[asse], label='Compensated', color='blue', alpha=0.8, linewidth=1.5)
        axs[2, i].plot(node.t_data, node.ship_orientation[asse], label='Ship', color='green', alpha=0.8, linewidth=1.5)
        axs[2, i].set_title(titoli_orientation[i])
        axs[2, i].set_xlabel('time (s)')
        axs[2, i].set_ylabel('radians')
        axs[2, i].grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    
    # Add global legend below the plots
    handles, labels = axs[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, -0.02), ncol=3, fontsize=11)
    
    # Save PDF if path is specified, otherwise show plot
    if node.pdf_output_path:
        try:
            output_file = Path(node.pdf_output_path)
            output_file.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(output_file, format='pdf', dpi=150, bbox_inches='tight')
            print(f"IMU comparison report saved to {output_file}.")
        except Exception as e:
            print(f"Failed to save PDF: {e}")
    else:
        plt.show()


def main(args=None):
    """Main entry point for the ROS2 node."""
    rclpy.init(args=args)
    node = ImuLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Stopping IMU Logger and plotting results...")
    finally:
        # Always close file and show plots, even when interrupted with Ctrl+C.
        if node.file:
            node.file.flush()
            node.file.close()
            print(f"Saved {node.sample_count} synchronized rows to '{node.csv_output_path}'.")
        
        if node.t_data:  # Only plot if we have data
            plot_results(node)
        
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # Context may already be shutdown by launcher
            pass


if __name__ == '__main__':
    main()
