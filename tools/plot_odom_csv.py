#!/usr/bin/env python3
"""
Script to read and plot odometry CSV data from odom_comparator output.
Generates comparison plots for ground truth vs estimated odometry.

Usage:
    python plot_odom_csv.py [path_to_csv] [--save output_image.png] [--pdf output_image.pdf]

- If no CSV path is provided, defaults to "odom_data.csv" in current directory.
- The optional --save flag saves the plot as PNG.
- The optional --pdf flag saves the plot as PDF.
- Plots position error and orientation error over time.
"""

import argparse
import csv
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np


def load_csv(csv_path: Path):
    """Load odometry CSV data."""
    data = {
        'time': [],
        'gt_pos_x': [], 'gt_pos_y': [], 'gt_pos_z': [],
        'est_pos_x': [], 'est_pos_y': [], 'est_pos_z': [],
        'err_pos_x': [], 'err_pos_y': [], 'err_pos_z': [],
        'err_ori_x': [], 'err_ori_y': [], 'err_ori_z': [],
        'err_lin_x': [], 'err_lin_y': [], 'err_lin_z': [],
        'err_ang_x': [], 'err_ang_y': [], 'err_ang_z': [],
        'cmd_speed': [],
    }
    
    try:
        with open(csv_path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    data['time'].append(float(row['time']))
                    data['gt_pos_x'].append(float(row['gt_pos_x']))
                    data['gt_pos_y'].append(float(row['gt_pos_y']))
                    data['gt_pos_z'].append(float(row['gt_pos_z']))
                    data['est_pos_x'].append(float(row['est_pos_x']))
                    data['est_pos_y'].append(float(row['est_pos_y']))
                    data['est_pos_z'].append(float(row['est_pos_z']))
                    data['err_pos_x'].append(float(row['err_pos_x']))
                    data['err_pos_y'].append(float(row['err_pos_y']))
                    data['err_pos_z'].append(float(row['err_pos_z']))
                    data['err_ori_x'].append(float(row['err_ori_x']))
                    data['err_ori_y'].append(float(row['err_ori_y']))
                    data['err_ori_z'].append(float(row['err_ori_z']))
                    data['err_lin_x'].append(float(row['err_lin_x']))
                    data['err_lin_y'].append(float(row['err_lin_y']))
                    data['err_lin_z'].append(float(row['err_lin_z']))
                    data['err_ang_x'].append(float(row['err_ang_x']))
                    data['err_ang_y'].append(float(row['err_ang_y']))
                    data['err_ang_z'].append(float(row['err_ang_z']))
                    data['cmd_speed'].append(float(row['cmd_speed']))
                except ValueError as e:
                    print(f"Warning: could not parse row: {e}")
                    continue
        
        return data
    
    except FileNotFoundError:
        print(f"Error: CSV file not found at {csv_path}")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        sys.exit(1)


def plot_odometry(data):
    """Generate odometry comparison plots."""
    if not data['time']:
        print("No data to plot")
        return
    
    t = np.array(data['time'])
    
    # Create figure with subplots
    fig, axs = plt.subplots(3, 2, figsize=(16, 12))
    fig.suptitle('Odometry Comparison: Ground Truth vs Estimated', fontsize=16, fontweight='bold')
    
    # === Row 1: Position Error ===
    # Position error X, Y, Z
    ax = axs[0, 0]
    ax.plot(t, data['err_pos_x'], label='X error', color='red', linewidth=1.5)
    ax.plot(t, data['err_pos_y'], label='Y error', color='green', linewidth=1.5)
    ax.plot(t, data['err_pos_z'], label='Z error', color='blue', linewidth=1.5)
    ax.set_title('Position Error (m)', fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (m)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Position error norm
    ax = axs[0, 1]
    err_pos_norm = np.sqrt(
        np.array(data['err_pos_x'])**2 + 
        np.array(data['err_pos_y'])**2 + 
        np.array(data['err_pos_z'])**2
    )
    ax.plot(t, err_pos_norm, color='purple', linewidth=1.5)
    ax.fill_between(t, err_pos_norm, alpha=0.3, color='purple')
    ax.set_title('Position Error Magnitude (m)', fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (m)')
    ax.grid(True, alpha=0.3)
    
    # === Row 2: Orientation Error ===
    # Orientation error X, Y, Z (radians)
    ax = axs[1, 0]
    ax.plot(t, data['err_ori_x'], label='Roll error', color='red', linewidth=1.5)
    ax.plot(t, data['err_ori_y'], label='Pitch error', color='green', linewidth=1.5)
    ax.plot(t, data['err_ori_z'], label='Yaw error', color='blue', linewidth=1.5)
    ax.set_title('Orientation Error (rad)', fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (rad)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Orientation error norm
    ax = axs[1, 1]
    err_ori_norm = np.sqrt(
        np.array(data['err_ori_x'])**2 + 
        np.array(data['err_ori_y'])**2 + 
        np.array(data['err_ori_z'])**2
    )
    ax.plot(t, err_ori_norm, color='orange', linewidth=1.5)
    ax.fill_between(t, err_ori_norm, alpha=0.3, color='orange')
    ax.set_title('Orientation Error Magnitude (rad)', fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (rad)')
    ax.grid(True, alpha=0.3)
    
    # === Row 3: Velocity Error ===
    # Linear velocity error
    ax = axs[2, 0]
    ax.plot(t, data['err_lin_x'], label='X velocity error', color='red', linewidth=1.5)
    ax.plot(t, data['err_lin_y'], label='Y velocity error', color='green', linewidth=1.5)
    ax.plot(t, data['err_lin_z'], label='Z velocity error', color='blue', linewidth=1.5)
    ax.set_title('Linear Velocity Error (m/s)', fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (m/s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Angular velocity error
    ax = axs[2, 1]
    ax.plot(t, data['err_ang_x'], label='X angular error', color='red', linewidth=1.5)
    ax.plot(t, data['err_ang_y'], label='Y angular error', color='green', linewidth=1.5)
    ax.plot(t, data['err_ang_z'], label='Z angular error', color='blue', linewidth=1.5)
    ax.set_title('Angular Velocity Error (rad/s)', fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (rad/s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Plot odometry CSV data from odom_comparator'
    )
    parser.add_argument(
        'csv_file',
        nargs='?',
        default='odom_data.csv',
        help='Path to CSV file (default: odom_data.csv)'
    )
    
    args = parser.parse_args()
    
    csv_path = Path(args.csv_file)
    
    if not csv_path.exists():
        print(f"Error: {csv_path} does not exist")
        sys.exit(1)
    
    print(f"Loading odometry data from {csv_path}...")
    data = load_csv(csv_path)
    
    if not data['time']:
        print("Error: No valid data in CSV file")
        sys.exit(1)
    
    print(f"Loaded {len(data['time'])} samples")
    print("Generating plots...")
    
    plot_odometry(data)


if __name__ == '__main__':
    main()
