#!/usr/bin/env python3

import time
import rclpy
import numpy as np
import csv
from pathlib import Path
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rtabmap_msgs.msg import OdomInfo
import message_filters
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from rclpy.qos import qos_profile_sensor_data
import matplotlib.gridspec as gridspec
from geometry_msgs.msg import Twist

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

A4_PORTRAIT_INCHES = (8.27, 11.69)


class OdomComparator(Node):

    def __init__(self):
        super().__init__('odom_comparator')

        self.declare_parameter('gt_robot_topic', '/robot/ground_truth/odom')
        self.declare_parameter('ship_joints_topic', '/ship/joint_states')
        self.declare_parameter('est_topic', '')
        self.declare_parameter('odom_info_topic', '/odom_info')
        self.declare_parameter('sync_slop', 0.05)
        self.declare_parameter('n_segments', 5)
        self.declare_parameter('csv_output_path', '')
        self.declare_parameter('pdf_output_path', '')

        self.sub_gt_robot    = message_filters.Subscriber(self, Odometry,  self.get_parameter('gt_robot_topic').value,   qos_profile=qos_profile_sensor_data)
        self.sub_ship_joints = message_filters.Subscriber(self, JointState, self.get_parameter('ship_joints_topic').value, qos_profile=qos_profile_sensor_data)
        self.sub_est         = message_filters.Subscriber(self, Odometry,  self.get_parameter('est_topic').value,        qos_profile=qos_profile_sensor_data)
        self.sub_odom_info   = message_filters.Subscriber(self, OdomInfo,  self.get_parameter('odom_info_topic').value,  qos_profile=qos_profile_sensor_data)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_gt_robot, self.sub_ship_joints, self.sub_est, self.sub_odom_info],
            queue_size=2000, slop=self.get_parameter('sync_slop').value)
        self.ts.registerCallback(self.sync_callback)

        self.start_time = None
        self.joint_idx  = None
        self.init_gt_pos,  self.init_est_pos  = None, None
        self.init_gt_rot,  self.init_est_rot  = None, None
        self.history = {
            't': [], 'gt': [], 'est': [],
            'e_pos': [], 'e_ori': [], 'e_lin': [], 'e_ang': [],
            'cmd_speed': [],
            'lost': [], 'inliers': [], 'matches': [], 'features': [],
        }

        self.cmd_speed = 0.0
        self.motion_started = False
        self._last_gt_pos = None        # gt_pos at previous sync tick, for motion detection
        self._last_motion_wall_t = None # wall-clock seconds of last sync tick where robot moved
        self.done = False
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def _vec(self, msg): return np.array([msg.x, msg.y, msg.z])
    def _quat(self, msg): return np.array([msg.x, msg.y, msg.z, msg.w])
    def wrap_angle(self, angle): return (angle + np.pi) % (2 * np.pi) - np.pi

    #############################################
    # Core callback to compare GT and estimation
    #############################################

    def cmd_vel_callback(self, msg):
        self.cmd_speed = np.sqrt(msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)
        if self.cmd_speed > 1e-3 and not self.motion_started:
            self.motion_started = True
            self.get_logger().info("Motion detected — waiting for first synchronized message to start tracking.")

    def sync_callback(self, msg_gt_robot, msg_ship_joints, msg_est, msg_odom_info):
        if not self.motion_started:
            return

        curr_t = msg_est.header.stamp.sec + msg_est.header.stamp.nanosec * 1e-9

        if self.joint_idx is None:
            names = msg_ship_joints.name
            self.joint_idx = [names.index(j) if j in names else -1
                              for j in ['heave_joint', 'roll_joint', 'pitch_joint']]

        p, v = msg_ship_joints.position, msg_ship_joints.velocity
        h_p, r_p, pi_p = [p[i] if i != -1 else 0.0 for i in self.joint_idx]
        h_v, r_v, pi_v = [v[i] if i != -1 else 0.0 for i in self.joint_idx]

        pos_robot_w = self._vec(msg_gt_robot.pose.pose.position)
        rot_robot_w = R.from_quat(self._quat(msg_gt_robot.pose.pose.orientation))

        ship_pos_w = np.array([0.0, 0.0, h_p])
        ship_rot_w = R.from_euler('XY', [r_p, pi_p])

        gt_pos = ship_rot_w.inv().apply(pos_robot_w - ship_pos_w)
        gt_rot = ship_rot_w.inv() * rot_robot_w

        # Detect robot motion from ground truth position change (wall time so it
        # works even when /clock freezes at end of bag).
        if self._last_gt_pos is not None and np.linalg.norm(gt_pos - self._last_gt_pos) > 5e-4:
            self._last_motion_wall_t = time.monotonic()
        self._last_gt_pos = gt_pos

        v_ship_w = np.array([0.0, 0.0, h_v]) + np.cross([r_v, pi_v, 0.0], pos_robot_w - ship_pos_w)
        inv_rot  = rot_robot_w.inv()
        gt_v_lin = self._vec(msg_gt_robot.twist.twist.linear)  - inv_rot.apply(v_ship_w)
        gt_v_ang = self._vec(msg_gt_robot.twist.twist.angular) - inv_rot.apply([r_v, pi_v, 0.0])

        est_pos = self._vec(msg_est.pose.pose.position)
        est_rot = R.from_quat(self._quat(msg_est.pose.pose.orientation))

        if self.start_time is None:
            self.start_time   = curr_t
            self.init_gt_pos, self.init_est_pos = gt_pos, est_pos
            self.init_gt_rot, self.init_est_rot = gt_rot, est_rot
            self.get_logger().info(f"Odometry tracking started at t={curr_t:.2f} s.")

        rel_gt_pos  = gt_pos  - self.init_gt_pos
        rel_est_pos = est_pos - self.init_est_pos
        euler_gt    = (self.init_gt_rot.inv() * gt_rot ).as_euler('xyz')
        euler_est   = (self.init_est_rot.inv() * est_rot).as_euler('xyz')

        self.history['t'].append(curr_t - self.start_time)
        self.history['gt'].append(rel_gt_pos)
        self.history['est'].append(rel_est_pos)
        self.history['e_pos'].append(rel_est_pos - rel_gt_pos)
        self.history['e_ori'].append(self.wrap_angle(euler_est - euler_gt))
        self.history['e_lin'].append(self._vec(msg_est.twist.twist.linear)  - gt_v_lin)
        self.history['e_ang'].append(self._vec(msg_est.twist.twist.angular) - gt_v_ang)
        self.history['cmd_speed'].append(self.cmd_speed)
        self.history['lost'].append(bool(msg_odom_info.lost))
        self.history['inliers'].append(int(msg_odom_info.inliers))
        self.history['matches'].append(int(msg_odom_info.matches))
        self.history['features'].append(int(msg_odom_info.features))

    ################
    # metrics 
    #################

    def _metrics_dict(self, e_pos, e_ori, e_lin, e_ang, gt_pos, cmd_speed=None, t=None):
        def stats(arr):
            ate = np.linalg.norm(arr, axis=1)
            return dict(rmse=np.sqrt(np.mean(ate**2)),
                        mean=np.mean(ate),
                        mx=np.max(ate),
                        final=ate[-1],
                        std=np.std(ate),
                        ate=ate)

        m = dict(pos=stats(e_pos), ori=stats(e_ori),
                 lin=stats(e_lin), ang=stats(e_ang))

        # Distance traveled: integrate the commanded speed (sampled at each sync tick)
        # using the accurate sim-time timestamps. This avoids inflating the distance
        # with sync-induced noise in gt_pos (position diffs) or with the oscillatory
        # Y/Z residuals left in gt_v_lin after imperfect ship-velocity cancellation.
        if cmd_speed is not None and t is not None and len(t) > 1:
            t_arr = np.asarray(t)
            dt_arr = np.diff(t_arr, prepend=t_arr[0])  # dt[0]=0 by convention
            dist = float(np.sum(np.asarray(cmd_speed) * dt_arr))
        elif len(gt_pos) > 1:
            step_distances = np.linalg.norm(np.diff(gt_pos, axis=0), axis=1)
            dist = np.sum(step_distances[step_distances > 0.005])
        else:
            dist = 1.0

        # Drift = medium ATE / distance traveled
        m['drift']    = m['pos']['mean'] / max(dist, 1e-3)
        m['dist']     = dist
        m['bias_pos'] = np.mean(e_pos, axis=0)
        m['bias_ori'] = np.mean(e_ori, axis=0)
        return m

    def _vo_health(self, lost_arr, inliers_arr, t):
        """VO tracking health from /odom_info.

        - drops:        rising-edge transitions of `lost` (each event = one failure)
        - lost_frames:  number of frames where `lost=True`
        - lost_ratio:   fraction of frames lost
        - max_outage_s: longest contiguous lost streak (seconds)
        - inliers_*:    inlier statistics (excluding lost frames)
        """
        n = len(lost_arr)
        if n == 0:
            return dict(drops=0, lost_frames=0, lost_ratio=0.0,
                        max_outage_s=0.0, inliers_mean=0.0, inliers_min=0)

        lost = np.asarray(lost_arr, dtype=bool)
        inliers = np.asarray(inliers_arr, dtype=float)
        t_arr = np.asarray(t, dtype=float)

        # Rising-edge events. If the first frame is lost, count it as a drop.
        d = np.diff(lost.astype(np.int8))
        drops = int(np.sum(d > 0)) + (1 if lost[0] else 0)

        lost_frames = int(np.sum(lost))
        lost_ratio  = lost_frames / n

        # Longest contiguous lost streak (in seconds, using median dt)
        max_streak_n = 0
        cur = 0
        for x in lost:
            cur = cur + 1 if x else 0
            if cur > max_streak_n:
                max_streak_n = cur
        med_dt = float(np.median(np.diff(t_arr))) if len(t_arr) > 1 else 0.05
        max_outage_s = max_streak_n * med_dt

        # Inlier stats over healthy frames only
        healthy = inliers[~lost] if (~lost).any() else inliers
        inliers_mean = float(np.mean(healthy)) if len(healthy) else 0.0
        inliers_min  = int(np.min(inliers)) if len(inliers) else 0

        return dict(drops=drops, lost_frames=lost_frames, lost_ratio=lost_ratio,
                    max_outage_s=max_outage_s,
                    inliers_mean=inliers_mean, inliers_min=inliers_min)
        
    @staticmethod
    def _panel(ax, title, color=None):
        color = color or C_ACCENT
        ax.set_facecolor(C_PANEL)
        for sp in ax.spines.values():
            sp.set_edgecolor(C_GRID)
        ax.tick_params(colors=C_SUBTEXT, labelsize=8)
        ax.set_title(title, color=color, fontsize=9, fontweight='bold', pad=6)
        ax.grid(True, color=C_GRID, linewidth=0.5, alpha=0.9)

    @staticmethod
    def _kv(ax, rows, x0=0.04, y0=0.88, dy=0.13, fs=8.5):
        for i, (k, v) in enumerate(rows):
            ax.text(x0, y0 - i * dy, k, transform=ax.transAxes,
                    fontsize=fs, color=C_SUBTEXT, va='top')
            ax.text(1 - x0, y0 - i * dy, v, transform=ax.transAxes,
                    fontsize=fs, color=C_TEXT, va='top', ha='right', fontweight='bold')
        ax.axis('off')
        ax.set_facecolor(C_PANEL)

    def plot_results(self):
        if not self.history['t']:
            return

        h = {k: np.array(v) for k, v in self.history.items()}

        m     = self._metrics_dict(h['e_pos'], h['e_ori'], h['e_lin'], h['e_ang'],
                                   h['gt'], h['cmd_speed'], h['t'])
        n_seg = self.get_parameter('n_segments').value
        t     = h['t']
        n     = len(t)
        seg_sz = n // n_seg

        seg_metrics = []
        for s in range(n_seg):
            i0 = s * seg_sz
            i1 = (s + 1) * seg_sz if s < n_seg - 1 else n
            seg_metrics.append(self._metrics_dict(
                h['e_pos'][i0:i1], h['e_ori'][i0:i1],
                h['e_lin'][i0:i1], h['e_ang'][i0:i1],
                h['gt'][i0:i1],
                h['cmd_speed'][i0:i1],
                h['t'][i0:i1]))

        # ── figure layout ──────────────────────────────────────────────────
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

        # ─────────────────────────────────────────────────────────────────
        # ROW 1:  trajectory · position global metrics · RPE
        # ─────────────────────────────────────────────────────────────────

        # --- 2D Trajectory ---
        ax_traj = fig.add_subplot(gs_top[0])
        self._panel(ax_traj, "2D Trajectory", color=C_ACCENT)
        ax_traj.plot(h['gt'][:, 0],  h['gt'][:, 1],  '--', color=C_TEAL,  lw=1.4, label='Ground truth', alpha=0.9)
        ax_traj.plot(h['est'][:, 0], h['est'][:, 1], '-',  color=C_CORAL, lw=1.4, label='Estimate',        alpha=0.9)
        ax_traj.set_xlabel('x  [m]', color=C_SUBTEXT, fontsize=8)
        ax_traj.set_ylabel('y  [m]', color=C_SUBTEXT, fontsize=8)
        ax_traj.legend(fontsize=7.5, facecolor=C_PANEL, edgecolor=C_GRID, labelcolor=C_TEXT)
        ax_traj.set_aspect('equal', adjustable='datalim')

        # Global metrics
        ax_tab = fig.add_subplot(gs_top[1])
        ax_tab.set_title("Global Metrics", color=C_ACCENT, fontsize=9, fontweight='bold', pad=6)
        p, o = m['pos'], m['ori']
        vo = self._vo_health(h['lost'], h['inliers'], t)
        rows = [
            ("— Position —",          ""),
            ("  RMSE",                 f"{p['rmse']:.4f} m"),
            ("  ATE mean",            f"{p['mean']:.4f} m"),
            ("  ATE max",          f"{p['mx']:.4f} m"),
            ("  Final error",        f"{p['final']:.4f} m"),
            ("  Std dev",              f"{p['std']:.4f} m"),
            ("  Drift (mean/dist)",    f"{m['drift']*100:.3f} %"),
            ("  Distance traveled",    f"{m['dist']:.2f} m"),
            ("— Orientation —",       ""),
            ("  RMSE",                 f"{o['rmse']:.4f} rad"),
            ("  ATE mean",            f"{o['mean']:.4f} rad"),
            ("  ATE max",          f"{o['mx']:.4f} rad"),
            ("  Final error",        f"{o['final']:.4f} rad"),
            ("— VO Health —",         ""),
            ("  Drops",                f"{vo['drops']}"),
            ("  Lost frames",          f"{vo['lost_frames']} / {len(t)}  ({vo['lost_ratio']*100:.2f} %)"),
            ("  Max outage",           f"{vo['max_outage_s']:.2f} s"),
            ("  Inliers (mean / min)", f"{vo['inliers_mean']:.0f} / {vo['inliers_min']}"),
        ]
        self._kv(ax_tab, rows, y0=0.94, dy=0.054, fs=8.0)


        # ─────────────────────────────────────────────────────────────────
        # ROW 2:  4 panels ATE over time
        # ─────────────────────────────────────────────────────────────────
        keys   = ['pos', 'ori', 'lin', 'ang']
        labels = ['Position [m]', 'Orientation [rad]', 'Linear Vel. [m/s]', 'Angular Vel. [rad/s]']
        colors = [C_CORAL, C_AMBER, C_TEAL, C_ACCENT]

        for col, (key, lbl, clr) in enumerate(zip(keys, labels, colors)):
            ax = fig.add_subplot(gs_mid[col])
            self._panel(ax, f"ATE {lbl}", color=clr)
            ate = m[key]['ate']
            ax.plot(t, ate, color=clr, lw=1.1, alpha=0.9)
            ax.fill_between(t, ate, alpha=0.12, color=clr)
            ax.axhline(m[key]['rmse'], color=C_TEXT, lw=0.9, linestyle='--',
                       alpha=0.5, label=f"RMSE {m[key]['rmse']:.4f}")
            ax.set_xlabel('t  [s]', color=C_SUBTEXT, fontsize=7)
            ax.legend(fontsize=6.5, facecolor=C_PANEL, edgecolor=C_GRID,
                      labelcolor=C_TEXT, loc='upper left')
            for s in range(1, n_seg):
                tx = t[min(s * seg_sz, n - 1)]
                ax.axvline(tx, color=C_GRID, lw=0.9, linestyle=':')
        # ─────────────────────────────────────────────────────────────────
        # ROW 3:  segment analysis
        # ─────────────────────────────────────────────────────────────────

        seg_labels = [
            f"S{i+1}\n{t[min(i*seg_sz, n-1)]:.0f}–{t[min((i+1)*seg_sz-1, n-1)]:.0f}s"
            for i in range(n_seg)]

        # RMSE position per segment
        ax_segp = fig.add_subplot(gs_bot[0, 0])
        self._panel(ax_segp, "RMSE position per segment  [m]", color=C_CORAL)
        vals_p = [sm['pos']['rmse'] for sm in seg_metrics]
        bars = ax_segp.bar(seg_labels, vals_p, color=C_CORAL,
                           edgecolor=C_GRID, linewidth=0.5, width=0.55, alpha=0.8)
        for bar, v in zip(bars, vals_p):
            ax_segp.text(bar.get_x() + bar.get_width()/2,
                         bar.get_height() + max(vals_p) * 0.01,
                         f"{v:.4f}", ha='center', va='bottom',
                         color=C_TEXT, fontsize=7.5, fontweight='bold')
        ax_segp.set_ylabel('RMSE [m]', color=C_SUBTEXT, fontsize=8)
        ax_segp.tick_params(axis='x', labelsize=7, labelcolor=C_SUBTEXT)
        ax_segp.tick_params(axis='y', labelsize=7, labelcolor=C_SUBTEXT)

        # RMSE orientation per segment
        ax_sego = fig.add_subplot(gs_bot[0, 1])
        self._panel(ax_sego, "RMSE orientation per segment  [rad]", color=C_AMBER)
        vals_o = [sm['ori']['rmse'] for sm in seg_metrics]
        bars = ax_sego.bar(seg_labels, vals_o, color=C_AMBER,
                           edgecolor=C_GRID, linewidth=0.5, width=0.55, alpha=0.8)
        for bar, v in zip(bars, vals_o):
            ax_sego.text(bar.get_x() + bar.get_width()/2,
                         bar.get_height() + max(vals_o) * 0.01,
                         f"{v:.4f}", ha='center', va='bottom',
                         color=C_TEXT, fontsize=7.5, fontweight='bold')
        ax_sego.set_ylabel('RMSE [rad]', color=C_SUBTEXT, fontsize=8)
        ax_sego.tick_params(axis='x', labelsize=7, labelcolor=C_SUBTEXT)
        ax_sego.tick_params(axis='y', labelsize=7, labelcolor=C_SUBTEXT)

        # Bias position per axis
        ax_bias = fig.add_subplot(gs_bot[1, 0])
        self._panel(ax_bias, "Bias position per axis  [m]", color=C_BLUE)
        bp = m['bias_pos']
        ax_bias.bar(['x', 'y', 'z'], bp, color=C_BLUE,
                    edgecolor=C_GRID, linewidth=0.5, width=0.4, alpha=0.8)
        ax_bias.axhline(0, color=C_SUBTEXT, lw=0.8)
        for i, v in enumerate(bp):
            ax_bias.text(i, v + np.sign(v) * max(abs(bp)) * 0.04,
                         f"{v:.4f}", ha='center',
                         va='bottom' if v >= 0 else 'top',
                         color=C_TEXT, fontsize=8, fontweight='bold')
        ax_bias.set_ylabel('Bias [m]', color=C_SUBTEXT, fontsize=8)
        ax_bias.tick_params(labelsize=9, labelcolor=C_SUBTEXT)

        # Bias orientation per axis
        ax_biao = fig.add_subplot(gs_bot[1, 1])
        self._panel(ax_biao, "Bias orientation per axis  [rad]", color=C_BLUE)
        bo = m['bias_ori']
        ax_biao.bar(['roll', 'pitch', 'yaw'], bo, color=C_BLUE,
                    edgecolor=C_GRID, linewidth=0.5, width=0.4, alpha=0.8)
        ax_biao.axhline(0, color=C_SUBTEXT, lw=0.8)
        for i, v in enumerate(bo):
            ax_biao.text(i, v + np.sign(v) * max(abs(bo)) * 0.04,
                         f"{v:.4f}", ha='center',
                         va='bottom' if v >= 0 else 'top',
                         color=C_TEXT, fontsize=8, fontweight='bold')
        ax_biao.set_ylabel('Bias [rad]', color=C_SUBTEXT, fontsize=8)
        ax_biao.tick_params(labelsize=9, labelcolor=C_SUBTEXT)

        plt.tight_layout()
        
        # Save PDF if path is specified, otherwise show plot
        pdf_path = self.get_parameter('pdf_output_path').value
        if pdf_path:
            try:
                output_file = Path(pdf_path)
                output_file.parent.mkdir(parents=True, exist_ok=True)
                fig.set_size_inches(*A4_PORTRAIT_INCHES, forward=True)
                fig.savefig(output_file, format='pdf', dpi=150, bbox_inches='tight')
                self.get_logger().info(f"Odometry report saved to {output_file}.")
            except Exception as e:
                self.get_logger().error(f"Failed to save PDF: {e}")
        else:
            # Show plot when not saving
            plt.show()
            plt.close(fig)

    def save_to_csv(self):
        """Save odometry tracking history to CSV file."""
        csv_path = self.get_parameter('csv_output_path').value
        
        if not csv_path:
            self.get_logger().info("csv_output_path not specified, skipping CSV export.")
            return
        
        if not self.history['t']:
            self.get_logger().warn("No data to save to CSV.")
            return
        
        try:
            # Ensure directory exists
            output_file = Path(csv_path)
            output_file.parent.mkdir(parents=True, exist_ok=True)
            
            # Prepare data for CSV
            h = {k: np.array(v) for k, v in self.history.items()}
            n_samples = len(h['t'])
            
            # Flatten position/orientation/velocity arrays for CSV
            fieldnames = ['time']
            
            # Ground truth position
            fieldnames.extend(['gt_pos_x', 'gt_pos_y', 'gt_pos_z'])
            # Estimated position
            fieldnames.extend(['est_pos_x', 'est_pos_y', 'est_pos_z'])
            # Position error
            fieldnames.extend(['err_pos_x', 'err_pos_y', 'err_pos_z'])
            # Orientation error (Euler)
            fieldnames.extend(['err_ori_x', 'err_ori_y', 'err_ori_z'])
            # Linear velocity error
            fieldnames.extend(['err_lin_x', 'err_lin_y', 'err_lin_z'])
            # Angular velocity error
            fieldnames.extend(['err_ang_x', 'err_ang_y', 'err_ang_z'])
            # Command speed
            fieldnames.append('cmd_speed')
            # VO health from /odom_info
            fieldnames.extend(['vo_lost', 'vo_inliers', 'vo_matches', 'vo_features'])
            
            with open(output_file, 'w', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                
                for i in range(n_samples):
                    row = {
                        'time': f"{h['t'][i]:.6f}",
                        'gt_pos_x': f"{h['gt'][i, 0]:.6f}",
                        'gt_pos_y': f"{h['gt'][i, 1]:.6f}",
                        'gt_pos_z': f"{h['gt'][i, 2]:.6f}",
                        'est_pos_x': f"{h['est'][i, 0]:.6f}",
                        'est_pos_y': f"{h['est'][i, 1]:.6f}",
                        'est_pos_z': f"{h['est'][i, 2]:.6f}",
                        'err_pos_x': f"{h['e_pos'][i, 0]:.6f}",
                        'err_pos_y': f"{h['e_pos'][i, 1]:.6f}",
                        'err_pos_z': f"{h['e_pos'][i, 2]:.6f}",
                        'err_ori_x': f"{h['e_ori'][i, 0]:.6f}",
                        'err_ori_y': f"{h['e_ori'][i, 1]:.6f}",
                        'err_ori_z': f"{h['e_ori'][i, 2]:.6f}",
                        'err_lin_x': f"{h['e_lin'][i, 0]:.6f}",
                        'err_lin_y': f"{h['e_lin'][i, 1]:.6f}",
                        'err_lin_z': f"{h['e_lin'][i, 2]:.6f}",
                        'err_ang_x': f"{h['e_ang'][i, 0]:.6f}",
                        'err_ang_y': f"{h['e_ang'][i, 1]:.6f}",
                        'err_ang_z': f"{h['e_ang'][i, 2]:.6f}",
                        'cmd_speed': f"{h['cmd_speed'][i]:.6f}",
                        'vo_lost':     int(h['lost'][i]),
                        'vo_inliers':  int(h['inliers'][i]),
                        'vo_matches':  int(h['matches'][i]),
                        'vo_features': int(h['features'][i]),
                    }
                    writer.writerow(row)
            
            self.get_logger().info(f"Odometry data saved to {output_file} ({n_samples} samples).")
        except Exception as e:
            self.get_logger().error(f"Failed to save CSV: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = OdomComparator()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            # Stop detection uses wall time so it fires even when /clock freezes
            # (bag finished). _last_motion_wall_t is set from gt_pos displacement
            # inside sync_callback, so it's independent of cmd_vel publication.
            if (node.motion_started
                    and node.start_time is not None
                    and node._last_motion_wall_t is not None
                    and (time.monotonic() - node._last_motion_wall_t) > 5.0):
                n = len(node.history['t'])
                curr_t = (node.history['t'][-1] + node.start_time) if n else 0.0
                node.get_logger().info(
                    f"Odometry tracking ended at t={curr_t:.2f} s "
                    f"({n} samples) — motion stopped for >5 s.")
                node.done = True
    except KeyboardInterrupt:
        n = len(node.history['t'])
        t_end = (node.history['t'][-1] + node.start_time) if n else 0.0
        print(f"[odom_comparator] Odometry tracking ended at t={t_end:.2f} s ({n} samples) — interrupted by user.")
    finally:
        node.save_to_csv()
        node.plot_results()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()