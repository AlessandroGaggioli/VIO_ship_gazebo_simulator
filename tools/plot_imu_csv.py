#!/usr/bin/env python3
"""
Offline IMU CSV plotter with layout matching imu_comparator.py.
Reads CSV output from imu_comparator node and displays comparison plots.

Usage:
    python plot_imu_csv.py [path_to_csv]
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

# Color palette matching imu_comparator.py
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

# Column groups for linear acceleration in each data stream.
ACC_COLUMNS = {
    "raw": ["raw_acc_x", "raw_acc_y", "raw_acc_z"],
    "comp": ["comp_acc_x", "comp_acc_y", "comp_acc_z"],
    "ship": ["ship_acc_x", "ship_acc_y", "ship_acc_z"],
}

# Column groups for angular velocity in each data stream.
OMEGA_COLUMNS = {
    "raw": ["raw_omega_x", "raw_omega_y", "raw_omega_z"],
    "comp": ["comp_omega_x", "comp_omega_y", "comp_omega_z"],
    "ship": ["ship_omega_x", "ship_omega_y", "ship_omega_z"],
}

# Shared visual style per stream
SERIES_STYLE = {
    "raw": {"label": "Raw", "color": "red", "alpha": 0.60},
    "comp": {"label": "Compensated", "color": "blue", "alpha": 0.80},
    "ship": {"label": "Ship", "color": "green", "alpha": 0.80},
}


def load_csv(csv_path: Path):
    # Store time and axis-wise values as lists for direct plotting.
    time_data = []
    acc_data = {k: [[], [], []] for k in ACC_COLUMNS}
    omega_data = {k: [[], [], []] for k in OMEGA_COLUMNS}
    orientation_data = {k: [[], [], []] for k in ['raw', 'comp', 'ship']}

    with csv_path.open("r", newline="") as file_obj:
        reader = csv.DictReader(file_obj)

        if reader.fieldnames is None:
            raise ValueError("CSV file does not have a valid header.")

        for row in reader:
            try:
                t = float(row["timestamp"])
            except (TypeError, ValueError, KeyError) as exc:
                raise ValueError("Invalid timestamp value in CSV.") from exc

            time_data.append(t)
            
            # Fill acceleration arrays in x, y, z order.
            for key, cols in ACC_COLUMNS.items():
                for axis_idx, col in enumerate(cols):
                    acc_data[key][axis_idx].append(float(row[col]))

            # Fill angular velocity arrays in x, y, z order.
            for key, cols in OMEGA_COLUMNS.items():
                for axis_idx, col in enumerate(cols):
                    omega_data[key][axis_idx].append(float(row[col]))
            
            # Fill orientation (roll, pitch, yaw)
            orientation_data['raw'][0].append(float(row['raw_roll']))
            orientation_data['raw'][1].append(float(row['raw_pitch']))
            orientation_data['raw'][2].append(float(row['raw_yaw']))
            orientation_data['comp'][0].append(float(row['comp_roll']))
            orientation_data['comp'][1].append(float(row['comp_pitch']))
            orientation_data['comp'][2].append(float(row['comp_yaw']))
            orientation_data['ship'][0].append(float(row['ship_roll']))
            orientation_data['ship'][1].append(float(row['ship_pitch']))
            orientation_data['ship'][2].append(float(row['ship_yaw']))

    if not time_data:
        raise ValueError("CSV contains no data rows.")

    # Convert absolute time to relative time
    t0 = time_data[0]
    rel_time = [t - t0 for t in time_data]

    return rel_time, acc_data, omega_data, orientation_data


def plot_data(time_data, acc_data, omega_data, orientation_data):
    """Generate IMU comparison plots matching imu_comparator.py layout."""
    axis_names = ["x", "y", "z"]
    axis_labels = ["X", "Y", "Z"]

    fig = plt.figure(figsize=(16, 11), facecolor=C_BG)
    gs = fig.add_gridspec(3, 3, hspace=0.5, wspace=0.4)
    fig.suptitle('Comparison IMU Robot: Raw (Red) vs Compensated (Blue) + Ship (Green)', fontsize=16)

    axs = np.empty((3, 3), dtype=object)
    for row in range(3):
        for col in range(3):
            axs[row, col] = fig.add_subplot(gs[row, col])
            axs[row, col].set_facecolor(C_PANEL)
            axs[row, col].grid(True, linestyle='--', alpha=0.7, color=C_GRID)

    # === Row 0: Linear Acceleration ===
    for i, axis in enumerate(axis_names):
        ax = axs[0, i]
        ax.plot(time_data, acc_data["raw"][i], label='Raw', color='red', alpha=0.6, linewidth=1.5)
        ax.plot(time_data, acc_data["comp"][i], label='Compensated', color='blue', alpha=0.8, linewidth=1.5)
        ax.plot(time_data, acc_data["ship"][i], label='Ship', color='green', alpha=0.8, linewidth=1.5)
        ax.set_title(f'Linear Acceleration {axis_labels[i]}', fontsize=10, fontweight='bold')
        if i == 0:
            ax.set_ylabel('m/s^2', fontsize=9)
        ax.tick_params(labelsize=8)

    # === Row 1: Angular Velocity ===
    for i, axis in enumerate(axis_names):
        ax = axs[1, i]
        ax.plot(time_data, omega_data["raw"][i], label='Raw', color='red', alpha=0.6, linewidth=1.5)
        ax.plot(time_data, omega_data["comp"][i], label='Compensated', color='blue', alpha=0.8, linewidth=1.5)
        ax.plot(time_data, omega_data["ship"][i], label='Ship', color='green', alpha=0.8, linewidth=1.5)
        ax.set_title(f'Angular Velocity {axis_labels[i]}', fontsize=10, fontweight='bold')
        if i == 0:
            ax.set_ylabel('rad/s', fontsize=9)
        ax.tick_params(labelsize=8)

    # === Row 2: Orientation (Roll, Pitch, Yaw) ===
    for i, axis in enumerate(axis_names):
        ax = axs[2, i]
        ax.plot(time_data, orientation_data["raw"][i], label='Raw', color='red', alpha=0.6, linewidth=1.5)
        ax.plot(time_data, orientation_data["comp"][i], label='Compensated', color='blue', alpha=0.8, linewidth=1.5)
        ax.plot(time_data, orientation_data["ship"][i], label='Ship', color='green', alpha=0.8, linewidth=1.5)
        ax.set_title(f'Orientation {axis_labels[i]}', fontsize=10, fontweight='bold')
        if i == 0:
            ax.set_ylabel('radians', fontsize=9)
        if i == 1:
            ax.set_xlabel('time (s)', fontsize=9)
        ax.tick_params(labelsize=8)

    # Add legend below plots
    handles, labels = axs[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=3, fontsize=11)

    fig.subplots_adjust(top=0.92, bottom=0.04, left=0.08)
    plt.show()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot IMU comparison CSV from imu_comparator."
    )
    parser.add_argument(
        "csv",
        nargs="?",
        default="data/imu_comparison.csv",
        help="Path to CSV file (default: data/imu_comparison.csv)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    csv_path = Path(args.csv)

    if not csv_path.exists():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    try:
        time_data, acc_data, omega_data, orientation_data = load_csv(csv_path)
    except ValueError as exc:
        print(f"Error: {exc}")
        sys.exit(1)

    print(f"Loaded {len(time_data)} IMU samples from {csv_path}")
    plot_data(time_data, acc_data, omega_data, orientation_data)


if __name__ == "__main__":
    main()
