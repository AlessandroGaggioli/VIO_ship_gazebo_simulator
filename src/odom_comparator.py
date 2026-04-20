#!/usr/bin/env python3

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

        self.declare_parameter('ekf_topic', '/odometry/filtered')
        self.declare_parameter('stereo_topic', '/stereo_odom')
        self.declare_parameter('sync_queue_size', 100)
        self.declare_parameter('sync_slop', 0.08)
        self.declare_parameter('report_period_sec', 1.0)
        self.declare_parameter('write_csv', False)
        self.declare_parameter('csv_path', '/home/alienware/ship_ws/bags/odom_comparison.csv')

        ekf_topic = self.get_parameter('ekf_topic').get_parameter_value().string_value
        stereo_topic = self.get_parameter('stereo_topic').get_parameter_value().string_value
        sync_queue_size = self.get_parameter('sync_queue_size').get_parameter_value().integer_value
        sync_slop = self.get_parameter('sync_slop').get_parameter_value().double_value
        report_period_sec = self.get_parameter('report_period_sec').get_parameter_value().double_value
        write_csv = self.get_parameter('write_csv').get_parameter_value().bool_value
        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value

        self.sub_ekf = message_filters.Subscriber(
            self,
            Odometry,
            ekf_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.sub_stereo = message_filters.Subscriber(
            self,
            Odometry,
            stereo_topic,
            qos_profile=qos_profile_sensor_data,
        )

        # pairs EKF and stereo messages with close timestamps (within sync_slop seconds) 
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_ekf, self.sub_stereo],
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
                'ekf_x',
                'ekf_y',
                'ekf_yaw',
                'ekf_vx',
                'ekf_vy',
                'ekf_wz',
                'stereo_x',
                'stereo_y',
                'stereo_yaw',
                'stereo_vx',
                'stereo_vy',
                'stereo_wz',
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
            f'Comparing EKF odom [{ekf_topic}] vs stereo odom [{stereo_topic}]'
        )

    def synced_callback(self, ekf_msg: Odometry, stereo_msg: Odometry):
        ekf_x = ekf_msg.pose.pose.position.x
        ekf_y = ekf_msg.pose.pose.position.y
        stereo_x = stereo_msg.pose.pose.position.x
        stereo_y = stereo_msg.pose.pose.position.y

        ex = ekf_x - stereo_x
        ey = ekf_y - stereo_y

        yaw_ekf = yaw_from_quaternion(ekf_msg.pose.pose.orientation)
        yaw_stereo = yaw_from_quaternion(stereo_msg.pose.pose.orientation)
        eyaw = wrap_angle(yaw_ekf - yaw_stereo)

        ekf_vx = ekf_msg.twist.twist.linear.x
        ekf_vy = ekf_msg.twist.twist.linear.y
        ekf_wz = ekf_msg.twist.twist.angular.z
        stereo_vx = stereo_msg.twist.twist.linear.x
        stereo_vy = stereo_msg.twist.twist.linear.y
        stereo_wz = stereo_msg.twist.twist.angular.z

        evx = ekf_vx - stereo_vx
        evy = ekf_vy - stereo_vy
        ewz = ekf_wz - stereo_wz

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
            stamp = ekf_msg.header.stamp.sec + 1e-9 * ekf_msg.header.stamp.nanosec
            self._csv_writer.writerow([
                stamp,
                ekf_x,
                ekf_y,
                yaw_ekf,
                ekf_vx,
                ekf_vy,
                ekf_wz,
                stereo_x,
                stereo_y,
                yaw_stereo,
                stereo_vx,
                stereo_vy,
                stereo_wz,
                ex,
                ey,
                eyaw,
                evx,
                evy,
                ewz,
            ])

    #print the mean error and RMSE for position, orientation, and velocity
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
            'N=%d | mean err: x=%.3f m y=%.3f m yaw=%.2f deg | '
            'RMSE: x=%.3f m y=%.3f m yaw=%.2f deg | '
            'STD: x=%.3f m y=%.3f m yaw=%.2f deg | '
            'VAR: x=%.5f y=%.5f yaw=%.5f rad^2 | '
            'mean vel err: vx=%.3f vy=%.3f wz=%.3f | '
            'RMSE vel: vx=%.3f vy=%.3f wz=%.3f | '
            'STD vel: vx=%.3f vy=%.3f wz=%.3f | '
            'VAR vel: vx=%.5f vy=%.5f wz=%.5f'
            % (
                self.count,
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
