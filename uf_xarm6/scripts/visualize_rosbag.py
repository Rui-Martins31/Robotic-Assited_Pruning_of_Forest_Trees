#!/usr/bin/env python3

import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from rosbags.rosbag2 import Reader

try:  # rosbags >= 0.10
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg

    _TS = get_typestore(Stores.ROS2_FOXY)

    def deserialize(rawdata, msgtype):
        return _TS.deserialize_cdr(rawdata, msgtype)

    def register_types(types):
        _TS.register(types)

except ImportError:  # rosbags 0.9.x (Jetson / Python 3.8)
    from rosbags.serde import deserialize_cdr as deserialize  # noqa: F401
    from rosbags.typesys import get_types_from_msg, register_types  # noqa: F401


REPO_ROOT = Path(__file__).resolve().parent.parent
OUTPUT_DIR = REPO_ROOT / 'ros_ws' / 'output'

CUSTOM_PACKAGES = ('xarm_msgs', 'control_msgs', 'moveit_msgs', 'object_recognition_msgs')
MSG_DIR_CANDIDATES = (
    REPO_ROOT / 'ros_ws' / 'src' / '{pkg}' / 'msg',
    REPO_ROOT / 'ros_ws' / 'install' / '{pkg}' / 'share' / '{pkg}' / 'msg',
    Path('/opt/ros/foxy/share') / '{pkg}' / 'msg',
)

TOPIC_CTRL_STATE = '/xarm6_traj_controller/state'
TOPIC_JOINT_STATES = '/joint_states'
TOPIC_FT_EXT = '/xarm/uf_ftsensor_ext_states'
TOPIC_FT_RAW = '/xarm/uf_ftsensor_raw_states'
TOPIC_PLANNED = '/display_planned_path'
TOPIC_ROBOT_STATES = '/xarm/robot_states'
TOPIC_COLLISION = '/monitoring/collision_caused_abnormal_current'
TOPIC_EXEC_EVENT = '/trajectory_execution_event'

ROBOT_STATES = {1: 'RUNNING', 2: 'SLEEPING', 3: 'PAUSED', 4: 'STOPPED', 5: 'CONFIG_CHANGED'}
ROBOT_MODES = {0: 'POSITION', 1: 'SERVOJ', 2: 'TEACHING_JOINT'}


# Bag discovery
def is_bag_dir(path):
    return (path / 'metadata.yaml').is_file() and any(path.glob('*.db3'))


def find_latest_bag(root):
    if is_bag_dir(root):
        return root
    bags = [p for p in root.rglob('*') if p.is_dir() and is_bag_dir(p)]
    return max(bags, key=lambda p: p.stat().st_mtime) if bags else None


def resolve_bag(argv):
    if len(argv) > 1:
        target = Path(argv[1]).expanduser()
        if not target.is_absolute():
            target = (Path.cwd() / target).resolve()
        if not target.exists():
            sys.exit(f'Path does not exist: {target}')
        bag = find_latest_bag(target)
        if bag is None:
            sys.exit(f'No rosbag found under: {target}')
        return bag

    if not OUTPUT_DIR.exists():
        sys.exit(f'Output directory not found: {OUTPUT_DIR}')
    bag = find_latest_bag(OUTPUT_DIR)
    if bag is None:
        sys.exit(f'No rosbags found under: {OUTPUT_DIR}')
    return bag


# Message type registration
def register_package(pkg):
    msg_dir = next(
        (Path(str(c).format(pkg=pkg)) for c in MSG_DIR_CANDIDATES
         if Path(str(c).format(pkg=pkg)).is_dir()),
        None,
    )
    if msg_dir is None:
        print(f'  [warn] {pkg}: msg/ directory not found; its topics are skipped')
        return

    types = {}
    for msg_file in sorted(msg_dir.glob('*.msg')):
        name = f'{pkg}/msg/{msg_file.stem}'
        try:
            types.update(get_types_from_msg(msg_file.read_text(), name))
        except Exception as exc:  # noqa: BLE001
            print(f'  [warn] {pkg}: failed to parse {msg_file.name}: {exc}')

    if types:
        try:
            register_types(types)
            print(f'  registered {pkg} ({len(types)} types)')
        except Exception as exc:  # noqa: BLE001
            print(f'  [warn] {pkg}: registration failed: {exc}')


def register_custom_types():
    print('Registering custom message types...')
    for pkg in CUSTOM_PACKAGES:
        register_package(pkg)


# Reading
def stamp_to_sec(header):
    try:
        return header.stamp.sec + header.stamp.nanosec * 1e-9
    except AttributeError:
        return None


class Series:
    def __init__(self):
        self.t = []
        self.rows = []

    def add(self, t, row):
        self.t.append(t)
        self.rows.append(row)

    def __len__(self):
        return len(self.t)

    def array(self):
        return np.asarray(self.t, dtype=float), np.asarray(self.rows, dtype=float)


def read_bag(bagpath):
    data = {
        'ctrl_actual_pos': Series(),
        'ctrl_desired_pos': Series(),
        'ctrl_error_pos': Series(),
        'ctrl_actual_vel': Series(),
        'ctrl_desired_vel': Series(),
        'js_effort': Series(),
        'ft_ext': Series(),
        'ft_raw': Series(),
        'robot_state': Series(),
        'planned': [],
        'events': {'collision': [], 'exec': []},
    }
    joint_names = {'ctrl': None, 'js': None}
    t0 = {'value': None}
    warned = set()

    def rel(bag_ns, header):
        t = stamp_to_sec(header) if header is not None else None
        if t is None:
            t = bag_ns * 1e-9
        if t0['value'] is None or t < t0['value']:
            t0['value'] = t
        return t

    with Reader(bagpath) as reader:
        for conn, ts_ns, raw in reader.messages():
            topic = conn.topic
            try:
                msg = deserialize(raw, conn.msgtype)
            except Exception as exc:  # noqa: BLE001
                if topic not in warned:
                    warned.add(topic)
                    print(f'  [warn] cannot decode {topic} ({conn.msgtype}): {exc}')
                continue

            if topic == TOPIC_CTRL_STATE:
                if joint_names['ctrl'] is None:
                    joint_names['ctrl'] = list(msg.joint_names)
                t = rel(ts_ns, getattr(msg, 'header', None))
                data['ctrl_actual_pos'].add(t, msg.actual.positions)
                data['ctrl_desired_pos'].add(t, msg.desired.positions)
                data['ctrl_error_pos'].add(t, msg.error.positions)
                if len(msg.actual.velocities):
                    data['ctrl_actual_vel'].add(t, msg.actual.velocities)
                if len(msg.desired.velocities):
                    data['ctrl_desired_vel'].add(t, msg.desired.velocities)

            elif topic == TOPIC_JOINT_STATES:
                if joint_names['js'] is None:
                    joint_names['js'] = list(msg.name)
                if len(msg.effort):
                    data['js_effort'].add(rel(ts_ns, getattr(msg, 'header', None)), msg.effort)

            elif topic == TOPIC_FT_EXT:
                data['ft_ext'].add(rel(ts_ns, getattr(msg, 'header', None)), _wrench_row(msg.wrench))

            elif topic == TOPIC_FT_RAW:
                data['ft_raw'].add(rel(ts_ns, getattr(msg, 'header', None)), _wrench_row(msg.wrench))

            elif topic == TOPIC_ROBOT_STATES:
                t = rel(ts_ns, getattr(msg, 'header', None))
                data['robot_state'].add(t, [msg.state, msg.mode, msg.err, msg.warn])

            elif topic == TOPIC_PLANNED:
                t = rel(ts_ns, getattr(msg, 'header', None))
                parsed = _parse_display_trajectory(msg)
                if parsed is not None:
                    data['planned'].append((t, *parsed))

            elif topic == TOPIC_COLLISION:
                data['events']['collision'].append((rel(ts_ns, None), 'collision'))

            elif topic == TOPIC_EXEC_EVENT:
                data['events']['exec'].append((rel(ts_ns, None), getattr(msg, 'data', 'event')))

    base = t0['value'] or 0.0
    for series in data.values():
        if isinstance(series, Series):
            series.t = [t - base for t in series.t]
    data['planned'] = [(t - base, names, pos) for (t, names, pos) in data['planned']]
    for kind in data['events']:
        data['events'][kind] = [(t - base, label) for (t, label) in data['events'][kind]]

    data['joint_names'] = joint_names['ctrl'] or joint_names['js'] or []
    return data


def _wrench_row(wrench):
    return [wrench.force.x, wrench.force.y, wrench.force.z,
            wrench.torque.x, wrench.torque.y, wrench.torque.z]


def _parse_display_trajectory(msg):
    try:
        segments = msg.trajectory
    except AttributeError:
        return None

    names, rows = None, []
    for seg in segments:
        jt = seg.joint_trajectory
        if not len(jt.points):
            continue
        if names is None:
            names = list(jt.joint_names)
        rows.extend(list(point.positions) for point in jt.points)

    if not rows or names is None:
        return None
    return names, np.asarray(rows, dtype=float)


# Plotting
def _grid(n):
    cols = 3 if n > 4 else min(n, 2)
    return int(np.ceil(n / cols)), cols


def plot_planned_vs_actual(data, joint_names, outdir, figs):
    actual = data['ctrl_actual_pos']
    if not len(actual):
        print('  [skip] planned-vs-actual: no controller state in bag')
        return

    n = len(joint_names)
    rows, cols = _grid(n)
    fig, axes = plt.subplots(rows, cols, figsize=(4 * cols, 2.6 * rows),
                             sharex=True, squeeze=False)
    fig.suptitle('Planned vs actual joint position')

    ta, pa = actual.array()
    td, pd = data['ctrl_desired_pos'].array() if len(data['ctrl_desired_pos']) else (None, None)

    for j, name in enumerate(joint_names):
        ax = axes[j // cols][j % cols]
        ax.plot(ta, pa[:, j], label='actual', color='tab:blue', lw=1.2)
        if td is not None:
            ax.plot(td, pd[:, j], label='desired', color='tab:orange', lw=1.0, ls='--')
        for (t, pnames, ppos) in data['planned']:
            if name in pnames:
                k = pnames.index(name)
                ax.scatter([t] * len(ppos), ppos[:, k], s=8, color='tab:green',
                           alpha=0.5, label='planned (MoveIt)')
        ax.set_title(name, fontsize=9)
        ax.set_ylabel('rad', fontsize=8)
        ax.grid(True, alpha=0.3)

    _dedup_legend(axes[0][0])
    _blank_unused(axes, n, cols)
    _finish(fig, axes, cols)
    _save(fig, outdir, 'planned_vs_actual_position.png', figs)


def plot_tracking_error(data, joint_names, outdir, figs):
    err = data['ctrl_error_pos']
    if not len(err):
        print('  [skip] tracking error: no controller state in bag')
        return
    t, e = err.array()
    fig, ax = plt.subplots(figsize=(10, 5))
    fig.suptitle('Joint position tracking error (desired - actual)')
    for j, name in enumerate(joint_names):
        ax.plot(t, e[:, j], label=name, lw=1.0)
    ax.axhline(0.0, color='k', lw=0.6, alpha=0.5)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Error [rad]')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, ncol=2)
    _save(fig, outdir, 'tracking_error.png', figs)


def plot_joint_effort(data, joint_names, outdir, figs):
    eff = data['js_effort']
    if not len(eff):
        print('  [skip] joint effort: no effort data in /joint_states')
        return
    t, e = eff.array()
    n = e.shape[1]
    names = joint_names[:n] if len(joint_names) >= n else [f'joint{i+1}' for i in range(n)]
    rows, cols = _grid(n)
    fig, axes = plt.subplots(rows, cols, figsize=(4 * cols, 2.6 * rows),
                             sharex=True, squeeze=False)
    fig.suptitle('Joint effort (torque)')
    for j in range(n):
        ax = axes[j // cols][j % cols]
        ax.plot(t, e[:, j], color='tab:red', lw=1.0)
        ax.set_title(names[j], fontsize=9)
        ax.set_ylabel('N·m', fontsize=8)
        ax.grid(True, alpha=0.3)
    _blank_unused(axes, n, cols)
    _finish(fig, axes, cols)
    _save(fig, outdir, 'joint_effort.png', figs)


def plot_joint_velocity(data, joint_names, outdir, figs):
    actual, desired = data['ctrl_actual_vel'], data['ctrl_desired_vel']
    if not len(actual) and not len(desired):
        print('  [skip] joint velocity: no velocity data in controller state')
        return

    ref = actual if len(actual) else desired
    n = ref.array()[1].shape[1]
    names = joint_names[:n] if len(joint_names) >= n else [f'joint{i+1}' for i in range(n)]
    rows, cols = _grid(n)
    fig, axes = plt.subplots(rows, cols, figsize=(4 * cols, 2.6 * rows),
                             sharex=True, squeeze=False)
    fig.suptitle('Joint velocity: actual vs desired')

    ta, va = actual.array() if len(actual) else (None, None)
    td, vd = desired.array() if len(desired) else (None, None)
    for j in range(n):
        ax = axes[j // cols][j % cols]
        if va is not None:
            ax.plot(ta, va[:, j], label='actual', color='tab:blue', lw=1.0)
        if vd is not None:
            ax.plot(td, vd[:, j], label='desired', color='tab:orange', lw=1.0, ls='--')
        ax.set_title(names[j], fontsize=9)
        ax.set_ylabel('rad/s', fontsize=8)
        ax.grid(True, alpha=0.3)
    _dedup_legend(axes[0][0])
    _blank_unused(axes, n, cols)
    _finish(fig, axes, cols)
    _save(fig, outdir, 'joint_velocity.png', figs)


def plot_ee_wrench(data, outdir, figs):
    ext, raw = data['ft_ext'], data['ft_raw']
    if not len(ext) and not len(raw):
        print('  [skip] EE wrench: no force/torque data in bag')
        return

    fig, (ax_f, ax_t) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    fig.suptitle('End-effector wrench (solid = ext/compensated, dashed = raw)')
    force_labels, torque_labels = ('Fx', 'Fy', 'Fz'), ('Tx', 'Ty', 'Tz')
    colors = ('tab:blue', 'tab:orange', 'tab:green')

    if len(ext):
        t, w = ext.array()
        for i in range(3):
            ax_f.plot(t, w[:, i], color=colors[i], lw=1.0, label=force_labels[i])
            ax_t.plot(t, w[:, 3 + i], color=colors[i], lw=1.0, label=torque_labels[i])
    if len(raw):
        t, w = raw.array()
        for i in range(3):
            ax_f.plot(t, w[:, i], color=colors[i], lw=0.8, ls='--', alpha=0.6)
            ax_t.plot(t, w[:, 3 + i], color=colors[i], lw=0.8, ls='--', alpha=0.6)

    ax_f.set_ylabel('Force [N]')
    ax_f.grid(True, alpha=0.3)
    ax_f.legend(fontsize=8, ncol=3)
    ax_t.set_ylabel('Torque [N·m]')
    ax_t.set_xlabel('Time [s]')
    ax_t.grid(True, alpha=0.3)
    ax_t.legend(fontsize=8, ncol=3)
    _save(fig, outdir, 'ee_wrench.png', figs)


def plot_robot_state_timeline(data, outdir, figs):
    rs = data['robot_state']
    if not len(rs):
        print('  [skip] robot state timeline: no /xarm/robot_states in bag')
        return

    t, rows = rs.array()
    state, mode, err, warn = rows[:, 0], rows[:, 1], rows[:, 2], rows[:, 3]

    fig, (ax_s, ax_m) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle('Robot state / mode timeline')

    ax_s.step(t, state, where='post', color='tab:blue', lw=1.2)
    ax_s.set_yticks(sorted(ROBOT_STATES))
    ax_s.set_yticklabels([ROBOT_STATES[k] for k in sorted(ROBOT_STATES)], fontsize=8)
    ax_s.set_ylabel('state')
    ax_s.grid(True, alpha=0.3)

    ax_m.step(t, mode, where='post', color='tab:purple', lw=1.2)
    ax_m.set_yticks(sorted(ROBOT_MODES))
    ax_m.set_yticklabels([ROBOT_MODES[k] for k in sorted(ROBOT_MODES)], fontsize=8)
    ax_m.set_ylabel('mode')
    ax_m.set_xlabel('Time [s]')
    ax_m.grid(True, alpha=0.3)

    for tk, ek, wk in zip(t, err, warn):
        if ek != 0:
            ax_s.axvline(tk, color='red', lw=0.8, alpha=0.6)
        if wk != 0:
            ax_s.axvline(tk, color='orange', lw=0.8, alpha=0.4)
    for (tk, _) in data['events']['collision']:
        ax_s.axvline(tk, color='crimson', lw=1.5, ls=':')
    for (tk, _) in data['events']['exec']:
        ax_m.axvline(tk, color='green', lw=1.0, ls=':')

    _save(fig, outdir, 'robot_state_timeline.png', figs)


def _dedup_legend(ax):
    handles, labels = ax.get_legend_handles_labels()
    seen = {}
    for h, l in zip(handles, labels):
        seen.setdefault(l, h)
    if seen:
        ax.legend(seen.values(), seen.keys(), fontsize=8)


def _blank_unused(axes, n, cols):
    for k in range(n, len(axes) * cols):
        axes[k // cols][k % cols].axis('off')


def _finish(fig, axes, cols):
    for c in range(cols):
        axes[-1][c].set_xlabel('Time [s]', fontsize=8)
    fig.tight_layout(rect=(0, 0, 1, 0.97))


def _save(fig, outdir, filename, figs):
    path = outdir / filename
    fig.savefig(path, dpi=120)
    print(f'  saved {path}')
    figs.append(fig)


# Main
def main(argv):
    bagpath = resolve_bag(argv)
    print(f'Using bag: {bagpath}')

    register_custom_types()

    print('Reading bag...')
    data = read_bag(bagpath)
    joint_names = data['joint_names']
    print(f'Joints: {joint_names or "(unknown)"}')
    print('Decoded message counts:')
    for key in ('ctrl_actual_pos', 'js_effort', 'ft_ext', 'ft_raw', 'robot_state'):
        print(f'  {key}: {len(data[key])}')
    print(f'  planned (MoveIt) trajectories: {len(data["planned"])}')

    if not joint_names:
        joint_names = [f'joint{i+1}' for i in range(6)]

    figs = []
    print('Plotting...')
    plot_planned_vs_actual(data, joint_names, bagpath, figs)
    plot_tracking_error(data, joint_names, bagpath, figs)
    plot_joint_effort(data, joint_names, bagpath, figs)
    plot_joint_velocity(data, joint_names, bagpath, figs)
    plot_ee_wrench(data, bagpath, figs)
    plot_robot_state_timeline(data, bagpath, figs)

    if figs:
        print(f'Showing {len(figs)} figure(s). Close the windows to exit.')
        plt.show()
    else:
        print('No figures were produced (the bag had no plottable data).')


if __name__ == '__main__':
    main(sys.argv)