#!/usr/bin/env python3
"""Aggregate odometry metrics across motion bags and (comp × 3dof) configs.

Expected directory layout under --path:

    <root>/
        motion_line/
            comp_off_3dof_off/   mot_line_..._A_odom.csv ... E_odom.csv
            comp_off_3dof_on/    ...
            comp_on_3dof_off/    ...
            comp_on_3dof_on/     ...
        motion_circle/           same structure
        motion_trajectory/       same structure
        motion_trajectory_speed/ same structure

Per-run metrics are averaged across the 5 runs of each config, then displayed
both on stdout (plain-text tables) and on a matplotlib figure (one table per
motion). If the CSVs include VO-health columns (vo_lost / vo_inliers / ...)
emitted by the updated odom_comparator, the VO summary columns are added too.
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

CONFIG_ORDER = [
    "comp_off_3dof_off",
    "comp_off_3dof_on",
    "comp_on_3dof_off",
    "comp_on_3dof_on",
]

C_HEADER_BG = "#3F4A8C"
C_HEADER_FG = "#FFFFFF"
C_ROW_A     = "#F4F4F8"
C_ROW_B     = "#FFFFFF"
C_GRID      = "#C8C8D8"


# --------------------------------------------------------------------------- #
# Per-run metrics                                                             #
# --------------------------------------------------------------------------- #

def metrics_from_csv(csv_path):
    """Compute scalar metrics from a single *_odom.csv. None on empty file."""
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fieldnames = reader.fieldnames or []

    if not rows:
        return None

    t     = np.array([float(r["time"])       for r in rows])
    e_pos = np.array([[float(r["err_pos_x"]), float(r["err_pos_y"]), float(r["err_pos_z"])] for r in rows])
    e_ori = np.array([[float(r["err_ori_x"]), float(r["err_ori_y"]), float(r["err_ori_z"])] for r in rows])
    cmd   = np.array([float(r["cmd_speed"])  for r in rows])

    ate_pos = np.linalg.norm(e_pos, axis=1)
    ate_ori = np.linalg.norm(e_ori, axis=1)

    if len(t) > 1:
        dt = np.diff(t, prepend=t[0])
        dist = float(np.sum(cmd * dt))
    else:
        dist = 1.0

    m = {
        "ate_mean":  float(np.mean(ate_pos)),
        "ate_max":   float(np.max(ate_pos)),
        "rmse_pos":  float(np.sqrt(np.mean(ate_pos ** 2))),
        "final_err": float(ate_pos[-1]),
        "drift":     float(np.mean(ate_pos) / max(dist, 1e-3) * 100.0),
        "rmse_ori":  float(np.sqrt(np.mean(ate_ori ** 2))),
        "dist":      dist,
        "has_vo":    False,
    }

    if "vo_lost" in fieldnames and "vo_inliers" in fieldnames:
        lost    = np.array([int(r["vo_lost"])    for r in rows], dtype=bool)
        inliers = np.array([int(r["vo_inliers"]) for r in rows], dtype=float)

        edges = np.diff(lost.astype(np.int8))
        drops = int(np.sum(edges > 0)) + (1 if lost[0] else 0)

        healthy = inliers[~lost] if (~lost).any() else inliers
        # longest contiguous lost streak (sec)
        max_streak, cur = 0, 0
        for x in lost:
            cur = cur + 1 if x else 0
            if cur > max_streak:
                max_streak = cur
        med_dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.05
        max_outage_s = max_streak * med_dt

        m["has_vo"]         = True
        m["vo_drops"]       = drops
        m["vo_lost_ratio"]  = float(np.mean(lost))
        m["vo_inliers_mean"] = float(np.mean(healthy)) if len(healthy) else 0.0
        m["vo_inliers_min"]  = int(np.min(inliers))
        m["vo_max_outage_s"] = max_outage_s

    return m


def aggregate_runs(metrics_list):
    """Mean + std per metric. Position metrics use ALL runs; VO metrics use
    only the subset of runs that actually carry VO columns (so old runs
    without /odom_info can be mixed with new instrumented runs)."""
    base_keys = ["ate_mean", "ate_max", "rmse_pos", "final_err",
                 "drift", "rmse_ori", "dist"]
    vo_keys   = ["vo_drops", "vo_lost_ratio", "vo_inliers_mean",
                 "vo_inliers_min", "vo_max_outage_s"]

    out = {}
    for k in base_keys:
        vals = np.array([m[k] for m in metrics_list], dtype=float)
        out[k]          = float(np.mean(vals))
        out[k + "_std"] = float(np.std(vals))

    vo_runs = [m for m in metrics_list if m.get("has_vo")]
    if vo_runs:
        for k in vo_keys:
            vals = np.array([m[k] for m in vo_runs], dtype=float)
            out[k]          = float(np.mean(vals))
            out[k + "_std"] = float(np.std(vals))
        out["has_vo"] = True
        out["vo_n"]   = len(vo_runs)
    else:
        out["has_vo"] = False
        out["vo_n"]   = 0

    out["n_runs"] = len(metrics_list)
    return out


# --------------------------------------------------------------------------- #
# Filesystem scan                                                             #
# --------------------------------------------------------------------------- #

def scan_motion_folder(motion_dir):
    """Walk one motion_* folder. Returns {config_name: aggregated metrics}."""
    result = {}
    for config_dir in sorted(motion_dir.iterdir()):
        if not config_dir.is_dir():
            continue
        csv_paths = sorted(config_dir.glob("*_odom.csv"))
        if not csv_paths:
            continue

        per_run = []
        for p in csv_paths:
            try:
                m = metrics_from_csv(p)
            except Exception as exc:
                print(f"warn: {p} skipped: {exc}", file=sys.stderr)
                continue
            if m is not None:
                per_run.append(m)

        if per_run:
            result[config_dir.name] = aggregate_runs(per_run)
    return result


def order_configs(configs):
    known   = [c for c in CONFIG_ORDER if c in configs]
    unknown = sorted(c for c in configs if c not in CONFIG_ORDER)
    return known + unknown


# --------------------------------------------------------------------------- #
# Output: terminal                                                            #
# --------------------------------------------------------------------------- #

def _ms(mean, std, p=4):
    """Format 'mean±std' with p decimals."""
    return f"{mean:.{p}f}±{std:.{p}f}"


def print_terminal_table(motion_name, cfg_metrics):
    has_vo = any(m.get("has_vo") for m in cfg_metrics.values())

    headers = ["Config", "N", "ATE mean[m]", "ATE max[m]", "RMSE pos[m]",
               "Final[m]", "Drift%", "RMSE ori[r]"]
    if has_vo:
        headers += ["VOn", "Drops", "Lost%", "Inliers", "MaxOut[s]"]

    table = []
    for cfg in order_configs(cfg_metrics.keys()):
        m = cfg_metrics[cfg]
        row = [
            cfg,
            f"{m['n_runs']}",
            _ms(m["ate_mean"],  m["ate_mean_std"]),
            _ms(m["ate_max"],   m["ate_max_std"]),
            _ms(m["rmse_pos"],  m["rmse_pos_std"]),
            _ms(m["final_err"], m["final_err_std"]),
            _ms(m["drift"],     m["drift_std"], 3),
            _ms(m["rmse_ori"],  m["rmse_ori_std"]),
        ]
        if has_vo:
            if m.get("has_vo"):
                row += [
                    f"{m['vo_n']}",
                    _ms(m["vo_drops"],                m["vo_drops_std"], 2),
                    _ms(m["vo_lost_ratio"] * 100.0,   m["vo_lost_ratio_std"] * 100.0, 2),
                    _ms(m["vo_inliers_mean"],         m["vo_inliers_mean_std"], 1),
                    _ms(m["vo_max_outage_s"],         m["vo_max_outage_s_std"], 2),
                ]
            else:
                row += ["0", "—", "—", "—", "—"]
        table.append(row)

    widths = [max(len(headers[i]), max(len(r[i]) for r in table))
              for i in range(len(headers))]
    total = sum(widths) + 2 * (len(widths) - 1)

    print()
    print("=" * total)
    print(f" {motion_name}")
    print("=" * total)
    print("  ".join(h.ljust(widths[0]) if i == 0 else h.rjust(widths[i])
                     for i, h in enumerate(headers)))
    print("-" * total)
    for r in table:
        print("  ".join(c.ljust(widths[0]) if i == 0 else c.rjust(widths[i])
                         for i, c in enumerate(r)))
    print("=" * total)


# --------------------------------------------------------------------------- #
# Output: matplotlib                                                          #
# --------------------------------------------------------------------------- #

def _figure_headers(has_vo):
    base = ["Config", "N",
            "ATE mean\n[m]", "ATE max\n[m]", "RMSE pos\n[m]",
            "Final err\n[m]", "Drift\n[%]", "RMSE ori\n[rad]"]
    if has_vo:
        base += ["VO\nn", "Drops\n/run", "Lost\n[%]", "Inliers\n(mean)",
                 "Max outage\n[s]"]
    return base


def _figure_row(cfg, m, has_vo):
    def ms(mean, std, p=4):
        return f"{mean:.{p}f}\n±{std:.{p}f}"

    row = [
        cfg,
        f"{m['n_runs']}",
        ms(m["ate_mean"],  m["ate_mean_std"]),
        ms(m["ate_max"],   m["ate_max_std"]),
        ms(m["rmse_pos"],  m["rmse_pos_std"]),
        ms(m["final_err"], m["final_err_std"]),
        ms(m["drift"],     m["drift_std"], 3),
        ms(m["rmse_ori"],  m["rmse_ori_std"]),
    ]
    if has_vo:
        if m.get("has_vo"):
            row += [
                f"{m['vo_n']}",
                ms(m["vo_drops"],              m["vo_drops_std"], 2),
                ms(m["vo_lost_ratio"] * 100,   m["vo_lost_ratio_std"] * 100, 2),
                ms(m["vo_inliers_mean"],       m["vo_inliers_mean_std"], 0),
                ms(m["vo_max_outage_s"],       m["vo_max_outage_s_std"], 2),
            ]
        else:
            row += ["0", "—", "—", "—", "—"]
    return row


def make_figure(all_results):
    n_motions = len(all_results)
    rows_total = sum(len(v) for v in all_results.values())
    fig_h = max(3.0, 1.4 * n_motions + 0.45 * rows_total)
    fig, axes = plt.subplots(n_motions, 1, figsize=(15, fig_h))
    if n_motions == 1:
        axes = [axes]
    fig.patch.set_facecolor("white")
    fig.suptitle("Odometry Metrics — Averages Across Runs",
                 fontsize=10, fontweight="bold", color="#1C1C2E")

    for ax, (motion, cfg_metrics) in zip(axes, all_results.items()):
        has_vo = any(m.get("has_vo") for m in cfg_metrics.values())
        headers = _figure_headers(has_vo)

        rows = [_figure_row(c, cfg_metrics[c], has_vo)
                for c in order_configs(cfg_metrics.keys())]

        ax.axis("off")
        # Title under each table
        n_runs = sorted({m["n_runs"] for m in cfg_metrics.values()})
        runs_label = f"{n_runs[0]}" if len(n_runs) == 1 else f"{min(n_runs)}–{max(n_runs)}"
        vo_ns = sorted({m["vo_n"] for m in cfg_metrics.values() if m.get("has_vo")})
        vo_lbl = ""
        if vo_ns:
            vo_lbl = (f", VO n={vo_ns[0]}" if len(vo_ns) == 1
                      else f", VO n={min(vo_ns)}–{max(vo_ns)}")
        ax.set_title(f"{motion}   (pos n={runs_label}{vo_lbl} — mean±std)",
                     fontsize=8, fontweight="bold", loc="left", color="#1C1C2E")

        tbl = ax.table(cellText=rows, colLabels=headers,
                       cellLoc="center", loc="center")
        tbl.auto_set_font_size(False)
        tbl.set_fontsize(7.0)
        tbl.scale(1, 2.4)

        # Header styling
        for j in range(len(headers)):
            tbl[(0, j)].set_facecolor(C_HEADER_BG)
            tbl[(0, j)].set_text_props(weight="bold", color=C_HEADER_FG)
            tbl[(0, j)].set_edgecolor(C_GRID)
        # Body styling
        for i in range(1, len(rows) + 1):
            for j in range(len(headers)):
                tbl[(i, j)].set_facecolor(C_ROW_A if i % 2 else C_ROW_B)
                tbl[(i, j)].set_edgecolor(C_GRID)
                tbl[(i, j)].set_text_props(color="#1C1C2E")

    plt.tight_layout()
    plt.subplots_adjust(top=0.95, hspace=0.6)
    plt.show()


# --------------------------------------------------------------------------- #
# Main                                                                        #
# --------------------------------------------------------------------------- #

def main():
    parser = argparse.ArgumentParser(
        description="Aggregate odom CSV metrics across motion bags × configs.")
    parser.add_argument(
        "--path",
        default="/home/alienware/ship_ws/SimulationResults",
        help="Root folder containing motion_* subfolders (default: %(default)s).")
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Print to terminal only — skip matplotlib window.")
    args = parser.parse_args()

    root = Path(args.path).expanduser().resolve()
    if not root.is_dir():
        print(f"error: folder not found: {root}", file=sys.stderr)
        sys.exit(1)

    motion_dirs = sorted(d for d in root.iterdir()
                         if d.is_dir() and d.name.startswith("motion_"))
    if not motion_dirs:
        print(f"error: no motion_* subfolders in {root}", file=sys.stderr)
        sys.exit(1)

    all_results = {}
    for md in motion_dirs:
        cfgs = scan_motion_folder(md)
        if cfgs:
            all_results[md.name] = cfgs
        else:
            print(f"warn: no CSV metrics found in {md}", file=sys.stderr)

    if not all_results:
        print("error: no metrics to display", file=sys.stderr)
        sys.exit(1)

    for motion, cfgs in all_results.items():
        print_terminal_table(motion, cfgs)
    print()

    if not args.no_plot:
        make_figure(all_results)


if __name__ == "__main__":
    main()
