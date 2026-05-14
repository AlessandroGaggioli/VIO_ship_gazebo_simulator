#!/usr/bin/env python3
"""
Offline odometry CSV plotter with layout matching odom_comparator.py.
Reads CSV output from odom_comparator node and displays evaluation plots.

Usage:
    python plot_odom_csv.py [path_to_csv]
"""

import argparse
import csv
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

# Color palette matching odom_comparator.py
C_BG        = "#FFFFFF"
C_PANEL     = "#F4F4F8"
C_ACCENT    = "#534AB7"
C_TEAL      = "#0F6E56"
C_CORAL     = "#993C1D"
C_AMBER     = "#854F0B"
C_BLUE      = "#185FA5"
C_TEXT      = "#1C1C2E"
C_SUBTEXT   = "#5A5A7A"
C_GRID      = "#DCDCE8"


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
    """Generate odometry evaluation plots matching odom_comparator.py."""
    if not data['time']:
        print("No data to plot")
        return
    
    t = np.array(data['time'])
    
    # Compute error norms
    err_pos_norm = np.sqrt(
        np.array(data['err_pos_x'])**2 + 
        np.array(data['err_pos_y'])**2 + 
        np.array(data['err_pos_z'])**2
    )
    err_ori_norm = np.sqrt(
        np.array(data['err_ori_x'])**2 + 
        np.array(data['err_ori_y'])**2 + 
        np.array(data['err_ori_z'])**2
    )
    
    # Create figure with three row groups
    fig = plt.figure(figsize=(18, 14), facecolor=C_BG)
    fig.suptitle("Odometry Evaluation Report", color=C_TEXT,
                 fontsize=16, fontweight='bold', y=0.98)

    gs_top = gridspec.GridSpec(1, 2, figure=fig,
                               top=0.93, bottom=0.68,
                               left=0.04, right=0.98, wspace=0.28)
    gs_mid = gridspec.GridSpec(1, 4, figure=fig,
                               top=0.63, bottom=0.38,
                               left=0.04, right=0.98, wspace=0.32)
    gs_bot = gridspec.GridSpec(2, 2, figure=fig,
                               top=0.33, bottom=0.04,
                               left=0.04, right=0.98,
                               hspace=0.42, wspace=0.28)

    # === ROW 1: 2D Trajectory & Position Error ===
    # 2D Trajectory
    ax = fig.add_subplot(gs_top[0])
    ax.set_facecolor(C_PANEL)
    ax.plot(data['gt_pos_x'], data['gt_pos_y'], '--', color=C_TEAL, lw=1.4, label='Ground truth', alpha=0.9)
    ax.plot(data['est_pos_x'], data['est_pos_y'], '-', color=C_CORAL, lw=1.4, label='Estimate', alpha=0.9)
    ax.set_xlabel('x  [m]', color=C_SUBTEXT, fontsize=8)
    ax.set_ylabel('y  [m]', color=C_SUBTEXT, fontsize=8)
    ax.set_title('2D Trajectory', color=C_ACCENT, fontsize=9, fontweight='bold', pad=6)
    ax.legend(fontsize=7.5, facecolor=C_PANEL, edgecolor=C_GRID, labelcolor=C_TEXT)
    ax.grid(True, color=C_GRID, linewidth=0.5, alpha=0.9)
    ax.set_aspect('equal', adjustable='datalim')

    # Position error magnitude
    ax = fig.add_subplot(gs_top[1])
    ax.set_facecolor(C_PANEL)
    ax.plot(t, err_pos_norm, color=C_CORAL, lw=1.1, alpha=0.9)
    ax.fill_between(t, err_pos_norm, alpha=0.12, color=C_CORAL)
    ax.set_xlabel('t  [s]', color=C_SUBTEXT, fontsize=7)
    ax.set_title(f'Position Error Magnitude [m]', color=C_CORAL, fontsize=9, fontweight='bold', pad=6)
    ax.legend(fontsize=6.5, facecolor=C_PANEL, edgecolor=C_GRID, labelcolor=C_TEXT, loc='upper left')
    ax.grid(True, color=C_GRID, linewidth=0.5, alpha=0.9)
    ax.tick_params(colors=C_SUBTEXT, labelsize=8)

    # === ROW 2: Error over time (4 panels) ===
    # Position X, Y, Z
    ax = fig.add_subplot(gs_mid[0])
    ax.set_facecolor(C_PANEL)
    ax.plot(t, data['err_pos_x'], label='X', color=C_CORAL, lw=1.1, alpha=0.9)
    ax.plot(t, data['err_pos_y'], label='Y', color=C_AMBER, lw=1.1, alpha=0.9)
    ax.plot(t, data['err_pos_z'], label='Z', color=C_TEAL, lw=1.1, alpha=0.9)
    ax.set_xlabel('t  [s]', color=C_SUBTEXT, fontsize=7)
    ax.set_title('Position Error [m]', color=C_ACCENT, fontsize=9, fontweight='bold', pad=6)
    ax.legend(fontsize=6.5, facecolor=C_PANEL, edgecolor=C_GRID, labelcolor=C_TEXT, loc='upper left')
    ax.grid(True, color=C_GRID, linewidth=0.5, alpha=0.9)
    ax.tick_params(colors=C_SUBTEXT, labelsize=8)

    # Orientation X, Y, Z
    ax = fig.add_subplot(gs_mid[1])
    ax.set_facecolor(C_PANEL)
    ax.plot(t, data['err_ori_x'], label='X', color=C_CORAL, lw=1.1, alpha=0.9)
    ax.plot(t, data['err_ori_y'], label='Y', color=C_AMBER, lw=1.1, alpha=0.9)
    ax.plot(t, data['err_ori_z'], label='Z', color=C_TEAL, lw=1.1, alpha=0.9)
    ax.set_xlabel('t  [s]', color=C_SUBTEXT, fontsize=7)
    ax.set_title('Orientation Error [rad]', color=C_ACCENT, fontsize=9, fontweight='bold', pad=6)
    ax.legend(fontsize=6.5, facecolor=C_PANEL, edgecolor=C_GRID, labelcolor=C_TEXT, loc='upper left')
    ax.grid(True, color=C_GRID, linewidth=0.5, alpha=0.9)
    ax.tick_params(colors=C_SUBTEXT, labelsize=8)

    # Linear velocity error
    ax = fig.add_subplot(gs_mid[2])
    ax.set_facecolor(C_PANEL)
    ax.plot(t, data['err_lin_x'], label='X', color=C_CORAL, lw=1.1, alpha=0.9)
    ax.plot(t, data['err_lin_y'], label='Y', color=C_AMBER, lw=1.1, alpha=0.9)
    ax.plot(t, data['err_lin_z'], label='Z', color=C_TEAL, lw=1.1, alpha=0.9)
    ax.set_xlabel('t  [s]', color=C_SUBTEXT, fontsize=7)
    ax.set_title('Linear Vel. Error [m/s]', color=C_ACCENT, fontsize=9, fontweight='bold', pad=6)
    ax.legend(fontsize=6.5, facecolor=C_PANEL, edgecolor=C_GRID, labelcolor=C_TEXT, loc='upper left')
    ax.grid(True, color=C_GRID, linewidth=0.5, alpha=0.9)
    ax.tick_params(colors=C_SUBTEXT, labelsize=8)

    # Angular velocity error
    ax = fig.add_subplot(gs_mid[3])
    ax.set_facecolor(C_PANEL)
    ax.plot(t, data['err_ang_x'], label='X', color=C_CORAL, lw=1.1, alpha=0.9)
    ax.plot(t, data['err_ang_y'], label='Y', color=C_AMBER, lw=1.1, alpha=0.9)
    ax.plot(t, data['err_ang_z'], label='Z', color=C_TEAL, lw=1.1, alpha=0.9)
    ax.set_xlabel('t  [s]', color=C_SUBTEXT, fontsize=7)
    ax.set_title('Angular Vel. Error [rad/s]', color=C_ACCENT, fontsize=9, fontweight='bold', pad=6)
    ax.legend(fontsize=6.5, facecolor=C_PANEL, edgecolor=C_GRID, labelcolor=C_TEXT, loc='upper left')
    ax.grid(True, color=C_GRID, linewidth=0.5, alpha=0.9)
    ax.tick_params(colors=C_SUBTEXT, labelsize=8)

    # === ROW 3: Statistics ===
    # Compute basic stats
    pos_rmse = np.sqrt(np.mean(err_pos_norm**2))
    ori_rmse = np.sqrt(np.mean(err_ori_norm**2))
    
    # Position bias per axis
    ax = fig.add_subplot(gs_bot[0, 0])
    ax.set_facecolor(C_PANEL)
    bias_pos = [np.mean(data['err_pos_x']), np.mean(data['err_pos_y']), np.mean(data['err_pos_z'])]
    bars = ax.bar(['x', 'y', 'z'], bias_pos, color=C_BLUE, edgecolor=C_GRID, linewidth=0.5, width=0.4, alpha=0.8)
    ax.axhline(0, color=C_SUBTEXT, lw=0.8)
    for i, v in enumerate(bias_pos):
        ax.text(i, v + np.sign(v) * max(abs(bias_pos)) * 0.04, f"{v:.4f}", ha='center',
                va='bottom' if v >= 0 else 'top', color=C_TEXT, fontsize=6.5, fontweight='bold')
    ax.set_ylabel('Bias [m]', color=C_SUBTEXT, fontsize=8)
    ax.set_title('Bias position per axis  [m]', color=C_BLUE, fontsize=9, fontweight='bold', pad=6)
    ax.tick_params(labelsize=9, labelcolor=C_SUBTEXT)
    ax.grid(True, color=C_GRID, linewidth=0.5, alpha=0.9, axis='y')

    # Orientation bias per axis
    ax = fig.add_subplot(gs_bot[0, 1])
    ax.set_facecolor(C_PANEL)
    bias_ori = [np.mean(data['err_ori_x']), np.mean(data['err_ori_y']), np.mean(data['err_ori_z'])]
    bars = ax.bar(['x', 'y', 'z'], bias_ori, color=C_BLUE, edgecolor=C_GRID, linewidth=0.5, width=0.4, alpha=0.8)
    ax.axhline(0, color=C_SUBTEXT, lw=0.8)
    for i, v in enumerate(bias_ori):
        ax.text(i, v + np.sign(v) * max(abs(bias_ori)) * 0.04, f"{v:.4f}", ha='center',
                va='bottom' if v >= 0 else 'top', color=C_TEXT, fontsize=6.5, fontweight='bold')
    ax.set_ylabel('Bias [rad]', color=C_SUBTEXT, fontsize=8)
    ax.set_title('Bias orientation per axis  [rad]', color=C_BLUE, fontsize=9, fontweight='bold', pad=6)
    ax.tick_params(labelsize=9, labelcolor=C_SUBTEXT)
    ax.grid(True, color=C_GRID, linewidth=0.5, alpha=0.9, axis='y')

    # Position stats
    ax = fig.add_subplot(gs_bot[1, 0])
    ax.axis('off')
    ax.set_facecolor(C_PANEL)
    ax.text(0.05, 0.85, 'Position Stats', transform=ax.transAxes, fontsize=9, fontweight='bold', color=C_TEXT)
    ax.text(0.05, 0.70, f"RMSE: {pos_rmse:.5f} m", transform=ax.transAxes, fontsize=8, color=C_SUBTEXT)
    ax.text(0.05, 0.55, f"Max: {np.max(err_pos_norm):.5f} m", transform=ax.transAxes, fontsize=8, color=C_SUBTEXT)
    ax.text(0.05, 0.40, f"Mean: {np.mean(err_pos_norm):.5f} m", transform=ax.transAxes, fontsize=8, color=C_SUBTEXT)
    
    # Orientation stats
    ax = fig.add_subplot(gs_bot[1, 1])
    ax.axis('off')
    ax.set_facecolor(C_PANEL)
    ax.text(0.05, 0.85, 'Orientation Stats', transform=ax.transAxes, fontsize=9, fontweight='bold', color=C_TEXT)
    ax.text(0.05, 0.70, f"RMSE: {ori_rmse:.5f} rad", transform=ax.transAxes, fontsize=8, color=C_SUBTEXT)
    ax.text(0.05, 0.55, f"Max: {np.max(err_ori_norm):.5f} rad", transform=ax.transAxes, fontsize=8, color=C_SUBTEXT)
    ax.text(0.05, 0.40, f"Mean: {np.mean(err_ori_norm):.5f} rad", transform=ax.transAxes, fontsize=8, color=C_SUBTEXT)

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
