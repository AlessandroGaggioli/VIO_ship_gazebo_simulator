#!/usr/bin/env python3

import os
import time
import csv
import rclpy
import numpy as np
import cv2
from pathlib import Path
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy,
                       qos_profile_sensor_data)
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist


class MapAnalyzer(Node):

    def __init__(self):
        super().__init__('map_analyzer')

        self.declare_parameter('map_name', 'map')
        self.declare_parameter('maps_dir', os.path.expanduser('~/ship_ws/maps'))
        self.declare_parameter('save_path', './analysis_results')
        self.declare_parameter('map_topic', '/grid_prob_map')
        self.declare_parameter('gt_robot_topic', '/robot/ground_truth/odom')
        self.declare_parameter('y_crop_min', 15)
        self.declare_parameter('y_crop_max', 78)

        # Latched QoS so we receive the latest map even after publication
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub_map = self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self.map_callback, map_qos)
        self.sub_gt = self.create_subscription(
            Odometry, self.get_parameter('gt_robot_topic').value,
            self.gt_callback, qos_profile_sensor_data)
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.map_msg = None
        self.cmd_speed = 0.0
        self.motion_started = False
        self._last_gt_pos = None         # gt position at previous tick, for motion detection
        self._last_motion_wall_t = None  # wall-clock seconds of last detected motion
        self.done = False

    #############################################
    # Callbacks (motion-stop detection mirrors odom_comparator)
    #############################################

    def cmd_vel_callback(self, msg):
        self.cmd_speed = np.sqrt(msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)
        if self.cmd_speed > 1e-3 and not self.motion_started:
            self.motion_started = True
            self.get_logger().info("Motion detected — waiting for map updates.")

    def gt_callback(self, msg):
        if not self.motion_started:
            return
        pos = np.array([msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z])
        if self._last_gt_pos is not None and np.linalg.norm(pos - self._last_gt_pos) > 5e-4:
            self._last_motion_wall_t = time.monotonic()
        self._last_gt_pos = pos

    def map_callback(self, msg):
        self.map_msg = msg

    #############################################
    # Map saving + metrics
    #############################################

    def process_map(self):
        """Build the Y-cropped PGM in memory, save it (+ YAML), compute
        Shannon entropy + coverage, append a row to map_metrics.csv."""
        if self.map_msg is None:
            self.get_logger().error("No /map message received — cannot save map.")
            return

        name = self.get_parameter('map_name').value
        maps_dir = self.get_parameter('maps_dir').value
        Path(maps_dir).mkdir(parents=True, exist_ok=True)

        w = self.map_msg.info.width
        h = self.map_msg.info.height
        resolution = self.map_msg.info.resolution
        origin = self.map_msg.info.origin

        data = np.array(self.map_msg.data, dtype=np.int16).reshape((h, w))

        # OccupancyGrid -> PGM grayscale (nav2 map_server convention)
        #   -1     -> 205 (unknown / gray)
        #   <= 25  -> 254 (free / white)   [free_thresh = 0.25]
        #   >= 65  -> 0   (occupied / black) [occupied_thresh = 0.65]
        pgm = np.full((h, w), 205, dtype=np.uint8)
        pgm[(data >= 0) & (data <= 25)] = 254
        pgm[(data >= 65) & (data <= 100)] = 0

        # OccupancyGrid origin is bottom-left, PGM is top-left -> flip vertically
        pgm = np.flipud(pgm)

        # Y-crop: keep only the band [y_min, y_max), rest = unknown gray
        y_min = max(0, min(self.get_parameter('y_crop_min').value, h))
        y_max = max(y_min, min(self.get_parameter('y_crop_max').value, h))
        filtered_map = np.full_like(pgm, 205)
        filtered_map[y_min:y_max, :] = pgm[y_min:y_max, :]

        pgm_path = os.path.join(maps_dir, f"{name}.pgm")
        yaml_path = os.path.join(maps_dir, f"{name}.yaml")

        cv2.imwrite(pgm_path, filtered_map)
        with open(yaml_path, 'w') as f:
            f.write(f"image: {name}.pgm\n")
            f.write(f"resolution: {resolution}\n")
            f.write(f"origin: [{origin.position.x}, {origin.position.y}, 0.0]\n")
            f.write("negate: 0\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.25\n")
        self.get_logger().info(f"Map saved to {pgm_path}")

        # Extract raw data and crop on y-axis 
        raw_data = np.array(self.map_msg.data, dtype=np.float64).reshape((h, w))
        raw_data = np.flipud(raw_data) 
        band_raw = raw_data[y_min:y_max, :]

        #probability map P(m)
        prob_map = np.full(band_raw.shape, 0.5, dtype=np.float64)
        known_mask = band_raw >= 0 #convert [0:100] to [0.0:1.0], unknown=-1 -> 0.5
        prob_map[known_mask] = np.clip(band_raw[known_mask] / 100.0, 0.01, 0.99) #clip 0.01 and 0.09 to avoid log(0)

        #entropy map H(m) = -p*log2(p) - (1-p)*log2(1-p)
        entropy_map = -(prob_map * np.log2(prob_map) + (1 - prob_map) * np.log2(1 - prob_map))

        explored_mask = (band_raw != -1)

        explored_entropy_sum = float(np.sum(entropy_map[explored_mask]))
        explored_cells = int(np.sum(explored_mask))
        total_cells = int(prob_map.size)

        mean_entropy = explored_entropy_sum / explored_cells if explored_cells > 0 else 0.0
        coverage = (explored_cells / total_cells) * 100.0 if total_cells > 0 else 0.0

        self.get_logger().info("--- MAP METRICS ---")
        self.get_logger().info(f"File:            {pgm_path}")
        self.get_logger().info(f"Total pixels:    {total_cells}")
        self.get_logger().info(f"Explored pixels: {explored_cells}")
        self.get_logger().info(f"Total entropy:   {explored_entropy_sum:.2f} bit")
        self.get_logger().info(f"Mean entropy:    {mean_entropy:.4f} bit/pixel")
        self.get_logger().info(f"Coverage:        {coverage:.2f}%")

        maps_dir = self.get_parameter('maps_dir').value
        csv_path = os.path.join(maps_dir, 'maps_metrics.csv')

        file_exists = os.path.exists(csv_path)
        with open(csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(['name', 'total_pixels', 'explored_pixels',
                                 'entropy_total', 'entropy_mean', 'coverage'])
            writer.writerow([name, total_cells, explored_cells,
                             f"{explored_entropy_sum:.4f}",
                             f"{mean_entropy:.6f}",
                             f"{coverage:.4f}"])

        self.get_logger().info(f"Metrics appended to {csv_path}")


def main(args=None):
    rclpy.init(args=args)
    node = MapAnalyzer()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            # Stop detection uses wall time so it fires even when /clock freezes
            # (bag finished). _last_motion_wall_t is set from gt_pos displacement
            # inside gt_callback, so it's independent of cmd_vel publication.
            if (node.motion_started
                    and node._last_motion_wall_t is not None
                    and (time.monotonic() - node._last_motion_wall_t) > 5.0):
                node.get_logger().info(
                    "Motion stopped for >5 s — saving map and computing metrics.")
                node.done = True
    except KeyboardInterrupt:
        print("[map_analyzer] Interrupted by user.")
    finally:
        node.process_map()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
