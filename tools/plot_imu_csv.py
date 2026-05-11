#!/usr/bin/env python3
"""
this script reads the CSV output from the IMU logger node (imu_comparison) and generates plots 
comparing raw, compensated, and ship data for both linear acceleration and angular velocity.
Usage:
    python plot_imu_csv.py [path_to_csv] [--save output_image.png]
- If no CSV path is provided, it defaults to "data/imu_comparison.csv" in the current directory.
- The optional --save flag allows you to specify a path to save the generated plot as an image file (e.g., PNG).
- The script validates the CSV format and will print clear error messages if required columns are missing or
if the file cannot be read. It also handles the case where the CSV contains no data rows, prompting the user to collect samples first.
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


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

# Shared visual style per stream so all subplots are consistent.
SERIES_STYLE = {
    "raw": {"label": "Raw", "color": "red", "alpha": 0.65},
    "comp": {"label": "Compensated", "color": "blue", "alpha": 0.85},
    "ship": {"label": "Ship", "color": "green", "alpha": 0.85},
}


def load_csv(csv_path: Path):
    # Store time and axis-wise values as lists for direct plotting.
    time_data = []
    acc_data = {k: [[], [], []] for k in ACC_COLUMNS}
    omega_data = {k: [[], [], []] for k in OMEGA_COLUMNS}

    with csv_path.open("r", newline="") as file_obj:
        reader = csv.DictReader(file_obj)

        if reader.fieldnames is None:
            raise ValueError("CSV file does not have a valid header.")

        # Validate schema early to fail with a clear message.
        missing = [
            col
            for col in ["timestamp"]
            + [c for cols in ACC_COLUMNS.values() for c in cols]
            + [c for cols in OMEGA_COLUMNS.values() for c in cols]
            if col not in reader.fieldnames
        ]
        if missing:
            raise ValueError(
                "CSV is missing required columns: " + ", ".join(missing)
            )

        for row in reader:
            try:
                t = float(row["timestamp"])
            except (TypeError, ValueError) as exc:
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

    if not time_data:
        raise ValueError("CSV contains no data rows.")

    # Convert absolute time to relative time for easier reading.
    t0 = time_data[0]
    rel_time = [t - t0 for t in time_data]

    return rel_time, acc_data, omega_data


def plot_data(time_data, acc_data, omega_data):
    axis_names = ["X", "Y", "Z"]

    # Grid layout: first row acceleration, second row angular velocity.
    fig, axs = plt.subplots(2, 3, figsize=(16, 9))
    fig.suptitle("IMU Comparison from CSV", fontsize=15)

    # Plot acceleration for X/Y/Z.
    for axis_idx, axis_name in enumerate(axis_names):
        ax = axs[0, axis_idx]
        for key, style in SERIES_STYLE.items():
            ax.plot(
                time_data,
                acc_data[key][axis_idx],
                label=style["label"],
                color=style["color"],
                alpha=style["alpha"],
                linewidth=1.6,
            )
        ax.set_title(f"Linear Acceleration {axis_name}")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("m/s^2")
        ax.grid(True, linestyle="--", alpha=0.6)

    # Plot angular velocity for X/Y/Z.
    for axis_idx, axis_name in enumerate(axis_names):
        ax = axs[1, axis_idx]
        for key, style in SERIES_STYLE.items():
            ax.plot(
                time_data,
                omega_data[key][axis_idx],
                label=style["label"],
                color=style["color"],
                alpha=style["alpha"],
                linewidth=1.6,
            )
        ax.set_title(f"Angular Velocity {axis_name}")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("rad/s")
        ax.grid(True, linestyle="--", alpha=0.6)

    plt.tight_layout()
    
    # Add global legend below the plots
    handles, labels = axs[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, -0.02), ncol=3, fontsize=11)
    
    plt.show()


def parse_args():
    # CLI supports an optional CSV path.
    parser = argparse.ArgumentParser(
        description="Read IMU comparison CSV and generate plots."
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

    # Guard against wrong paths before attempting to parse the file.
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    try:
        time_data, acc_data, omega_data = load_csv(csv_path)
    except ValueError as exc:
        print(f"Error: {exc}")
        if "no data rows" in str(exc).lower():
            print(
                "Tip: start your IMU logger first to collect samples, then run this plot script again."
            )
        sys.exit(1)

    plot_data(time_data, acc_data, omega_data)


if __name__ == "__main__":
    main()
