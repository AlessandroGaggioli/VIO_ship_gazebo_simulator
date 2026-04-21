#!/usr/bin/env python3

"""

Odometry comparator for comparing estimated and ground truth odometry.

This node subscribes to two odometry topics (e.g., from stereo odometry and ground truth) and computes the error between them.
It periodically reports the mean error, RMSE, and standard deviation for position, orientation, and velocity. 
Optionally, it can also write the synchronized odometry data and errors to a CSV file for further analysis.
"""

import csv
import math
from pathlib import Path

import message_filters
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def yaw_from_quaternion(q):
    # Standard yaw extraction from quaternion.
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle): # normalize angle to [-pi, pi]
    return math.atan2(math.sin(angle), math.cos(angle))


def variance_from_sums(sum_value, sum_sq_value, count):
    mean = sum_value / count
    return max(0.0, (sum_sq_value / count) - (mean * mean))


class OdomComparator(Node):
    def __init__(self):
        super().__init__('odom_comparator')

        self.declare_parameter('target_topic', '/stereo_odom')
        self.declare_parameter('truth_topic', '/ground_truth/odom')
        self.declare_parameter('target_name', 'target')
        self.declare_parameter('truth_name', 'truth')
        self.declare_parameter('sync_queue_size', 100)
        self.declare_parameter('sync_slop', 0.08)
        self.declare_parameter('report_period_sec', 1.0)
        self.declare_parameter('write_csv', False)
        self.declare_parameter('csv_path', '/home/alienware/ship_ws/bags/odom_comparison.csv')

        target_topic = self.get_parameter('target_topic').get_parameter_value().string_value
        truth_topic = self.get_parameter('truth_topic').get_parameter_value().string_value
        self.target_name = self.get_parameter('target_name').get_parameter_value().string_value
        self.truth_name = self.get_parameter('truth_name').get_parameter_value().string_value
        sync_queue_size = self.get_parameter('sync_queue_size').get_parameter_value().integer_value
        sync_slop = self.get_parameter('sync_slop').get_parameter_value().double_value
        report_period_sec = self.get_parameter('report_period_sec').get_parameter_value().double_value
        write_csv = self.get_parameter('write_csv').get_parameter_value().bool_value
        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value

        self.sub_target = message_filters.Subscriber(
            self,
            Odometry,
            target_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.sub_truth = message_filters.Subscriber(
            self,
            Odometry,
            truth_topic,
            qos_profile=qos_profile_sensor_data,
        )

        # Pairs estimated odometry and ground-truth messages with close timestamps.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_target, self.sub_truth],
            queue_size=int(sync_queue_size),
            slop=float(sync_slop),
        )
        self.sync.registerCallback(self.synced_callback)

        self.timer = self.create_timer(float(report_period_sec), self.report)

        self.count = 0
        self.sum_err = {
            'x': 0.0,
            'y': 0.0,
            'yaw': 0.0,
            'vx': 0.0,
            'vy': 0.0,
            'wz': 0.0,
        }
        self.sum_sq_err = {
            'x': 0.0,
            'y': 0.0,
            'yaw': 0.0,
            'vx': 0.0,
            'vy': 0.0,
            'wz': 0.0,
        }

        self._csv_file = None
        self._csv_writer = None
        if write_csv:
            csv_file_path = Path(csv_path)
            csv_file_path.parent.mkdir(parents=True, exist_ok=True)
            self._csv_file = csv_file_path.open('w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            csv_header = [
                't',
                'target_x',
                'target_y',
                'target_yaw',
                'target_vx',
                'target_vy',
                'target_wz',
                'truth_x',
                'truth_y',
                'truth_yaw',
                'truth_vx',
                'truth_vy',
                'truth_wz',
                'ex',
                'ey',
                'eyaw',
                'evx',
                'evy',
                'ewz',
            ]
            self._csv_writer.writerow(csv_header)
            self.get_logger().info(
                'CSV output enabled -> %s | columns: %s'
                % (csv_file_path, ','.join(csv_header))
            )

        self.get_logger().info(
            f'Comparing target [{target_topic}] vs truth [{truth_topic}]'
        )

    def synced_callback(self, target_msg: Odometry, truth_msg: Odometry):
        target_x = target_msg.pose.pose.position.x
        target_y = target_msg.pose.pose.position.y
        truth_x = truth_msg.pose.pose.position.x
        truth_y = truth_msg.pose.pose.position.y

        ex = target_x - truth_x
        ey = target_y - truth_y

        yaw_target = yaw_from_quaternion(target_msg.pose.pose.orientation)
        yaw_truth = yaw_from_quaternion(truth_msg.pose.pose.orientation)
        eyaw = wrap_angle(yaw_target - yaw_truth)

        target_vx = target_msg.twist.twist.linear.x
        target_vy = target_msg.twist.twist.linear.y
        target_wz = target_msg.twist.twist.angular.z
        truth_vx = truth_msg.twist.twist.linear.x
        truth_vy = truth_msg.twist.twist.linear.y
        truth_wz = truth_msg.twist.twist.angular.z

        evx = target_vx - truth_vx
        evy = target_vy - truth_vy
        ewz = target_wz - truth_wz

        errors = {
            'x': ex,
            'y': ey,
            'yaw': eyaw,
            'vx': evx,
            'vy': evy,
            'wz': ewz,
        }

        self.count += 1
        for key, value in errors.items():
            self.sum_err[key] += value
            self.sum_sq_err[key] += value * value

        if self._csv_writer is not None:
            stamp = target_msg.header.stamp.sec + 1e-9 * target_msg.header.stamp.nanosec
            self._csv_writer.writerow([
                stamp,
                target_x,
                target_y,
                yaw_target,
                target_vx,
                target_vy,
                target_wz,
                truth_x,
                truth_y,
                yaw_truth,
                truth_vx,
                truth_vy,
                truth_wz,
                ex,
                ey,
                eyaw,
                evx,
                evy,
                ewz,
            ])

    # Print the mean error and RMSE for position, orientation and velocity.
    def report(self):
        if self.count == 0:
            self.get_logger().warn('No synchronized odometry samples yet.')
            return

        mean_x = self.sum_err['x'] / self.count
        mean_y = self.sum_err['y'] / self.count
        mean_yaw = self.sum_err['yaw'] / self.count
        mean_vx = self.sum_err['vx'] / self.count
        mean_vy = self.sum_err['vy'] / self.count
        mean_wz = self.sum_err['wz'] / self.count

        rmse_x = math.sqrt(self.sum_sq_err['x'] / self.count)
        rmse_y = math.sqrt(self.sum_sq_err['y'] / self.count)
        rmse_yaw = math.sqrt(self.sum_sq_err['yaw'] / self.count)
        rmse_vx = math.sqrt(self.sum_sq_err['vx'] / self.count)
        rmse_vy = math.sqrt(self.sum_sq_err['vy'] / self.count)
        rmse_wz = math.sqrt(self.sum_sq_err['wz'] / self.count)

        var_x = variance_from_sums(self.sum_err['x'], self.sum_sq_err['x'], self.count)
        var_y = variance_from_sums(self.sum_err['y'], self.sum_sq_err['y'], self.count)
        var_yaw = variance_from_sums(self.sum_err['yaw'], self.sum_sq_err['yaw'], self.count)
        var_vx = variance_from_sums(self.sum_err['vx'], self.sum_sq_err['vx'], self.count)
        var_vy = variance_from_sums(self.sum_err['vy'], self.sum_sq_err['vy'], self.count)
        var_wz = variance_from_sums(self.sum_err['wz'], self.sum_sq_err['wz'], self.count)

        std_x = math.sqrt(var_x)
        std_y = math.sqrt(var_y)
        std_yaw = math.sqrt(var_yaw)
        std_vx = math.sqrt(var_vx)
        std_vy = math.sqrt(var_vy)
        std_wz = math.sqrt(var_wz)

        self.get_logger().info(
            'N=%d | %s-%s mean err: x=%.3f m y=%.3f m yaw=%.2f deg | '
            'RMSE: x=%.3f m y=%.3f m yaw=%.2f deg | '
            'STD: x=%.3f m y=%.3f m yaw=%.2f deg | '
            'VAR: x=%.5f y=%.5f yaw=%.5f rad^2 | '
            'mean vel err: vx=%.3f vy=%.3f wz=%.3f | '
            'RMSE vel: vx=%.3f vy=%.3f wz=%.3f | '
            'STD vel: vx=%.3f vy=%.3f wz=%.3f | '
            'VAR vel: vx=%.5f vy=%.5f wz=%.5f'
            % (
                self.count,
                self.target_name,
                self.truth_name,
                mean_x,
                mean_y,
                math.degrees(mean_yaw),
                rmse_x,
                rmse_y,
                math.degrees(rmse_yaw),
                std_x,
                std_y,
                math.degrees(std_yaw),
                var_x,
                var_y,
                var_yaw,
                mean_vx,
                mean_vy,
                mean_wz,
                rmse_vx,
                rmse_vy,
                rmse_wz,
                std_vx,
                std_vy,
                std_wz,
                var_vx,
                var_vy,
                var_wz,
            )
        )

    def destroy_node(self):
        if self._csv_file is not None:
            self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomComparator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
