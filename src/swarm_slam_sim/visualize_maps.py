#!/usr/bin/env python3
"""
visualize_maps.py  —  Swarm-SLAM map comparison tool

Plots three panels:
  1. Ground truth    (robot odometry)
  2. CSLAM baseline  (optimized keyframe trajectory, map-frame aligned)
  3. CSLAM + RL      (same, for later comparison)

Key fix: aligns Swarm-SLAM's internal map frame to the world frame
by matching each robot's first keyframe to its first odom position.

Usage:
  # Single recording (GT + CSLAM):
  python3 visualize_maps.py ~/ROS2_SWARM/map_recordings/baseline_*/

  # Compare baseline vs RL:
  python3 visualize_maps.py \
      --baseline ~/ROS2_SWARM/map_recordings/baseline_*/ \
      --rl       ~/ROS2_SWARM/map_recordings/rl_*/

  # Save figure:
  python3 visualize_maps.py <dir> --save output.png

  # Skip slow point cloud rendering:
  python3 visualize_maps.py <dir> --no-clouds
"""

import sys, os, json, argparse, math
import numpy as np

# ── Robot config ─────────────────────────────────────────────────────────────
ROBOT_COLORS = {
    'robot_0': '#C8442A',   # warm red
    'robot_1': '#1A8C5C',   # teal green
    'robot_2': '#1A5A9C',   # slate blue
}
ROBOT_LABELS = {
    'robot_0': 'R0  (redundant path)',
    'robot_1': 'R1  (diverse path)',
    'robot_2': 'R2  (diverse path)',
}
ROBOTS = ['robot_0', 'robot_1', 'robot_2']

# Building bounds (world frame)
BLD_X = (-30, 30)
BLD_Y = (-22.5, 22.5)


# ─────────────────────────────────────────────────────────────────────────────
# I/O
# ─────────────────────────────────────────────────────────────────────────────

def load_recording(directory):
    def _load(fname):
        path = os.path.join(directory, fname)
        return json.load(open(path)) if os.path.exists(path) else None
    return {
        'ground_truth':    _load('ground_truth.json'),
        'cslam_estimates': _load('cslam_estimates.json'),
        'clouds':          _load('cslam_pointclouds.json'),
        'dir':             directory,
    }


# ─────────────────────────────────────────────────────────────────────────────
# Frame alignment
# ─────────────────────────────────────────────────────────────────────────────

def compute_map_to_world_offset(gt_data, cslam_data):
    """
    Swarm-SLAM anchors its map frame to the robot's first keyframe.
    The robot's first odom position is its spawn point in world frame.
    Offset = first_odom_pos - first_keyframe_pos
    
    Returns dict: robot_name -> (dx, dy)
    """
    offsets = {}
    if not gt_data or not cslam_data:
        return offsets

    for robot in ROBOTS:
        gt_poses = gt_data.get(robot, [])
        cs_poses = cslam_data.get(robot, {})
        if not gt_poses or not cs_poses:
            continue

        # First odom position (world frame)
        first_odom_x = gt_poses[0]['x']
        first_odom_y = gt_poses[0]['y']

        # First keyframe pose (map frame) — sort by keyframe_id
        sorted_kfs = sorted(cs_poses.items(), key=lambda x: int(x[0]))
        first_kf = sorted_kfs[0][1]
        first_kf_x = first_kf['x']
        first_kf_y = first_kf['y']

        dx = first_odom_x - first_kf_x
        dy = first_odom_y - first_kf_y
        offsets[robot] = (dx, dy)

    return offsets


def transform_poses(cs_poses, offset):
    """
    Apply map→world offset to keyframe poses.
    Also filter:
      - Poses outside building bounds
      - Poses that jump more than MAX_JUMP from the previous one
        (unconverged optimizer artifacts)
    """
    MAX_JUMP = 8.0   # metres — anything larger is an outlier
    dx, dy   = offset

    sorted_kfs = sorted(cs_poses.items(), key=lambda x: int(x[0]))
    result = []
    prev = None

    for kf_id, pose in sorted_kfs:
        wx = pose['x'] + dx
        wy = pose['y'] + dy

        # Building bounds filter
        if not (BLD_X[0] - 5 <= wx <= BLD_X[1] + 5 and
                BLD_Y[0] - 5 <= wy <= BLD_Y[1] + 5):
            continue

        # Jump filter — skip if too far from last valid pose
        if prev is not None:
            dist = math.sqrt((wx - prev[0])**2 + (wy - prev[1])**2)
            if dist > MAX_JUMP:
                continue

        result.append((wx, wy))
        prev = (wx, wy)

    return result


# ─────────────────────────────────────────────────────────────────────────────
# Metrics
# ─────────────────────────────────────────────────────────────────────────────

def compute_coverage(poses, cell_size=2.0):
    """Fraction of building cells covered by a trajectory."""
    if not poses:
        return 0.0
    nx = int((BLD_X[1] - BLD_X[0]) / cell_size)
    ny = int((BLD_Y[1] - BLD_Y[0]) / cell_size)
    visited = set()
    for x, y in poses:
        cx = int((x - BLD_X[0]) / cell_size)
        cy = int((y - BLD_Y[0]) / cell_size)
        if 0 <= cx < nx and 0 <= cy < ny:
            visited.add((cx, cy))
    return len(visited) / (nx * ny)


def compute_ate(gt_poses, cslam_poses):
    """Mean distance between closest GT point and each CSLAM keyframe."""
    if not gt_poses or not cslam_poses:
        return None
    gt_arr = np.array([(p['x'], p['y']) for p in gt_poses])
    errors = []
    for wx, wy in cslam_poses:
        dists = np.sqrt(((gt_arr - [wx, wy])**2).sum(axis=1))
        errors.append(dists.min())
    return float(np.mean(errors)) if errors else None


def compute_path_length(poses):
    if not poses:
        return 0.0
    total = 0.0
    for i in range(1, len(poses)):
        dx = poses[i][0] - poses[i-1][0]
        dy = poses[i][1] - poses[i-1][1]
        total += math.sqrt(dx*dx + dy*dy)
    return total


# ─────────────────────────────────────────────────────────────────────────────
# Drawing helpers
# ─────────────────────────────────────────────────────────────────────────────

def draw_building(ax):
    import matplotlib.patches as patches

    # Floor
    ax.add_patch(patches.Rectangle(
        (-30, -22.5), 60, 45,
        facecolor='#F2F0EC', edgecolor='#888', linewidth=1.2, zorder=0))

    W = dict(color='#AAA', linewidth=0.8, zorder=1)

    # Corridor walls at y=±3.5 with door gap at x=-12 to -9.5
    for y in [3.5, -3.5]:
        ax.plot([-30, -12], [y, y], **W)
        ax.plot([-9.5,  6], [y, y], **W)

    # Vertical divider at x=6 with door gaps
    ax.plot([6, 6], [22.5, 16.5], **W)
    ax.plot([6, 6], [14,    3.5], **W)
    ax.plot([6, 6], [1,    -3.5], **W)
    ax.plot([6, 6], [-7.5, -10],  **W)
    ax.plot([6, 6], [-10, -22.5], **W)

    # Office/Lab divider y=10.5 (right side)
    ax.plot([6, 30], [10.5,  10.5], **W)
    # Lab/Storage at y=-3.5 right side
    ax.plot([6, 30], [-3.5, -3.5],  **W)

    # Door markers (green ticks)
    D = dict(color='#4CAF80', linewidth=1.2, zorder=2)
    for dx in [-12, -9.5]:
        ax.plot([dx, dx], [3.5-0.6, 3.5+0.6],  **D)
        ax.plot([dx, dx], [-3.5-0.6, -3.5+0.6], **D)
    for dy in [14, 16.5]:
        ax.plot([6-0.6, 6+0.6], [dy, dy], **D)
    for dy in [1, 3.5]:
        ax.plot([6-0.6, 6+0.6], [dy, dy], **D)
    for dy in [-7.5, -10]:
        ax.plot([6-0.6, 6+0.6], [dy, dy], **D)

    # Room labels
    LK = dict(ha='center', va='center', fontsize=7,
               color='#CCC', style='italic', zorder=2)
    ax.text(-12,  13,    'Large Bay',  **LK)
    ax.text( 18,  16.5,  'Office',     **LK)
    ax.text(-12,   0,    'Corridor',   **LK)
    ax.text( 18,   3.5,  'Lab',        **LK)
    ax.text(-12, -13,    'Workshop',   **LK)
    ax.text( 18, -13,    'Storage',    **LK)


def style_ax(ax, title, subtitle=''):
    ax.set_xlim(-33, 33)
    ax.set_ylim(-25, 25)
    ax.set_aspect('equal')
    ax.set_xlabel('x  (m)', fontsize=8)
    ax.set_ylabel('y  (m)', fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(True, color='#E8E8E8', linewidth=0.4, zorder=0)
    full = title if not subtitle else f'{title}\n{subtitle}'
    ax.set_title(full, fontsize=9, pad=6)


def plot_gt(ax, data):
    """Ground truth odometry trajectories."""
    gt = data.get('ground_truth') or {}
    for robot in ROBOTS:
        poses = gt.get(robot, [])
        if not poses:
            continue
        xs = [p['x'] for p in poses]
        ys = [p['y'] for p in poses]
        c  = ROBOT_COLORS[robot]
        ax.plot(xs, ys, '-', color=c, lw=1.3, alpha=0.85,
                label=ROBOT_LABELS[robot], zorder=4)
        ax.plot(xs[0],  ys[0],  'o', color=c, ms=7,  zorder=6)
        ax.plot(xs[-1], ys[-1], 's', color=c, ms=5,  zorder=6)


def plot_cslam(ax, data, offsets, show_clouds=False):
    """CSLAM optimized trajectory with frame alignment and outlier removal."""
    cs = data.get('cslam_estimates') or {}
    gt = data.get('ground_truth')    or {}

    all_ate    = []
    stats_lines = []

    for robot in ROBOTS:
        cs_poses_raw = cs.get(robot, {})
        if not cs_poses_raw:
            continue

        offset = offsets.get(robot, (0.0, 0.0))
        poses  = transform_poses(cs_poses_raw, offset)
        if not poses:
            continue

        c  = ROBOT_COLORS[robot]
        xs = [p[0] for p in poses]
        ys = [p[1] for p in poses]

        ax.plot(xs, ys, '-', color=c, lw=1.1, alpha=0.80, zorder=4)
        ax.scatter(xs, ys, s=5, color=c, alpha=0.50, zorder=5)
        ax.plot(xs[0], ys[0], 'o', color=c, ms=7, zorder=6)

        # ATE vs ground truth
        gt_list = gt.get(robot, [])
        ate     = compute_ate(gt_list, poses)
        cov     = compute_coverage(poses)
        plen    = compute_path_length(poses)
        if ate is not None:
            all_ate.append(ate)
        r = robot.replace('robot_', 'R')
        line = (f'{r}: ATE={ate:.2f}m  cov={cov*100:.0f}%  '
                f'len={plen:.0f}m'
                if ate is not None else f'{r}: no data')
        stats_lines.append(line)

    return stats_lines


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    pa = argparse.ArgumentParser()
    pa.add_argument('recording_dir', nargs='?')
    pa.add_argument('--baseline')
    pa.add_argument('--rl')
    pa.add_argument('--save')
    pa.add_argument('--no-clouds', action='store_true')
    args = pa.parse_args()

    try:
        import matplotlib
        if args.save:
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.lines import Line2D
    except ImportError:
        print('pip3 install matplotlib numpy')
        sys.exit(1)

    show_clouds = False  # disabled until proper SE2 transforms implemented

    # ── Load data ─────────────────────────────────────────────────────────
    if args.baseline and args.rl:
        bl  = load_recording(args.baseline)
        rl  = load_recording(args.rl)
        ncols = 3
        mode  = 'compare'
    elif args.recording_dir:
        bl   = load_recording(args.recording_dir)
        rl   = None
        ncols = 2
        mode  = 'single'
    else:
        print('Usage: visualize_maps.py <dir>  OR  '
              '--baseline <dir> --rl <dir>')
        sys.exit(1)

    # Compute frame offsets
    bl_offsets = compute_map_to_world_offset(
        bl.get('ground_truth'), bl.get('cslam_estimates'))
    rl_offsets = compute_map_to_world_offset(
        rl.get('ground_truth'), rl.get('cslam_estimates')) if rl else {}

    # ── Figure ────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(1, ncols, figsize=(7*ncols, 7.5))
    fig.patch.set_facecolor('#FAFAF8')
    fig.suptitle('Swarm-SLAM  ·  Map Comparison',
                 fontsize=13, fontweight='500', y=0.98)

    # Panel 1 — Ground Truth
    ax = axes[0]
    draw_building(ax)
    plot_gt(ax, bl)
    style_ax(ax, 'Ground Truth', 'robot odometry (world frame)')
    ax.legend(fontsize=7, loc='lower left',
              framealpha=0.8, edgecolor='#DDD')

    # Panel 2 — CSLAM Baseline
    ax = axes[1]
    draw_building(ax)
    stats = plot_cslam(ax, bl, bl_offsets, show_clouds)
    subtitle = '\n'.join(stats) if stats else 'no CSLAM data'
    style_ax(ax, 'Swarm-SLAM  Baseline', subtitle)

    # Panel 3 — CSLAM + RL (if provided)
    if mode == 'compare' and rl:
        ax = axes[2]
        draw_building(ax)
        stats_rl = plot_cslam(ax, rl, rl_offsets, show_clouds)
        subtitle_rl = '\n'.join(stats_rl) if stats_rl else 'no CSLAM data'
        style_ax(ax, 'Swarm-SLAM  +  RL', subtitle_rl)

    # Shared legend
    legend_elems = [
        Line2D([0], [0], color=ROBOT_COLORS[r], lw=2,
               label=ROBOT_LABELS[r])
        for r in ROBOTS
    ]
    legend_elems += [
        Line2D([0], [0], marker='o', color='#888', ms=6,
               linestyle='none', label='Start'),
        Line2D([0], [0], marker='s', color='#888', ms=5,
               linestyle='none', label='End'),
    ]
    fig.legend(handles=legend_elems, loc='lower center',
               ncol=len(legend_elems), fontsize=8,
               bbox_to_anchor=(0.5, 0.01), framealpha=0.9,
               edgecolor='#DDD')

    plt.tight_layout(rect=[0, 0.06, 1, 0.96])

    if args.save:
        plt.savefig(args.save, dpi=150, bbox_inches='tight',
                    facecolor=fig.get_facecolor())
        print(f'Saved → {args.save}')
    else:
        plt.show()


if __name__ == '__main__':
    main()
