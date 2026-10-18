#!/usr/bin/env python3
"""Replay recorded TF/point clouds and report opening tracking statistics, not accuracy."""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import secrets
import signal
import sqlite3
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from lidar_perception_system.msg import MovingBucketTrack, BucketBottom
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

from replay_static_tf import databases, read_static_tf


def bag_start_ns(path):
    starts = []
    for database in databases(path):
        with sqlite3.connect(database.resolve().as_uri() + '?mode=ro', uri=True) as connection:
            start = connection.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0]
            if start is not None:
                starts.append(start)
    if not starts:
        raise ValueError('The bag contains no messages')
    return min(starts)


def stop(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path, help='rosbag2 SQLite directory or .db3 file')
    parser.add_argument('--config', type=Path, required=True, help='JSON with explicit opponent_roi')
    parser.add_argument('--offset', type=float, default=936.)
    parser.add_argument('--seconds', type=float, default=60.)
    parser.add_argument('--warmup-seconds', type=float, default=0.,
                        help='Replay this many seconds before offset, retaining the learned model')
    parser.add_argument('--recompute-localization', action='store_true',
                        help='Launch gn10_pointcloud_localization using recorded LiDAR/IMU only')
    parser.add_argument('--baseline-executable', type=Path,
                        help='Optional existing CUDA moving_bucket_detector executable')
    parser.add_argument('--baseline-config', type=Path, default=Path(get_package_share_directory(
        'lidar_perception_system')) / 'config' / 'moving_bucket.json')
    parser.add_argument('--output', type=Path, default=Path('/tmp/lps_opening_benchmark.json'))
    args = parser.parse_args()
    if (not args.bag.exists() or not all(math.isfinite(x) for x in
            (args.offset, args.seconds, args.warmup_seconds)) or args.offset < 0
            or args.seconds <= 0 or not 0 <= args.warmup_seconds <= args.offset):
        parser.error('bag must exist, seconds must be positive, and 0 <= warmup <= offset')
    config = json.loads(args.config.read_text())
    if not isinstance(config.get('opponent_roi'), dict):
        parser.error('config must specify opponent_roi')
    if args.baseline_executable and not args.baseline_executable.is_file():
        parser.error('baseline executable does not exist')
    if args.recompute_localization:
        get_package_share_directory('gn10_pointcloud_localization')
        static_message = None
        replay_topics = ['/livox/lidar', '/livox/imu']
    else:
        static_message = read_static_tf(args.bag)
        replay_topics = [config['input_topic'], '/tf', '/tf_static']
    evaluation_start_ns = bag_start_ns(args.bag) + round(args.offset * 1e9)
    evaluation_end_ns = evaluation_start_ns + round(args.seconds * 1e9)

    def in_evaluation(stamp):
        ns = stamp.sec * 10**9 + stamp.nanosec
        return evaluation_start_ns <= ns < evaluation_end_ns
    run_dir = Path(tempfile.mkdtemp(prefix='lps_opening_'))
    env = os.environ.copy()
    # Each run is isolated from a robot's existing ROS domain and command consumers.
    env['ROS_DOMAIN_ID'] = str(20 + secrets.randbelow(180))
    env['ROS_LOCALHOST_ONLY'] = '1'
    env['ROS_LOG_DIR'] = str(run_dir / 'ros_logs')
    os.environ.update({key: env[key] for key in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'ROS_LOG_DIR')})
    rclpy.init()
    node = rclpy.create_node('opening_benchmark', parameter_overrides=[Parameter('use_sim_time', value=True)])
    static_pub = node.create_publisher(TFMessage, '/tf_static', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    if static_message is not None:
        static_pub.publish(static_message)
    streams = {}
    diagnostic = {}
    sampled_peaks = {}
    reasons = {}
    subscriptions = []
    cloud_count = [0]
    bottom_observations = {}
    def bottom(message):
        if not in_evaluation(message.header.stamp):
            return
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if message.valid:
            bottom_observations[stamp] = message.track_id
    subscriptions.append(node.create_subscription(BucketBottom,
        '/bucket_opening_tracker/bottom', bottom, 100))
    subscriptions.append(node.create_subscription(PointCloud2, config['input_topic'],
        lambda m: cloud_count.__setitem__(0, cloud_count[0] + int(in_evaluation(m.header.stamp))),
        rclpy.qos.qos_profile_sensor_data))
    prefixes = {'opening': '/bucket_opening_tracker'}
    if args.baseline_executable:
        prefixes['baseline'] = '/moving_bucket_detector'
    for name, prefix in prefixes.items():
        streams[name] = {'raw': {}, 'final_valid': set(), 'final_invalid_reports': 0}
        reasons[name] = Counter()
        def raw(message, name=name):
            if not in_evaluation(message.header.stamp):
                return
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            if stamp > 0:
                streams[name]['raw'][stamp] = (message.valid, message.observation_mode, message.track_id)
        def final(message, name=name):
            if not in_evaluation(message.header.stamp):
                return
            if message.valid:
                stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
                streams[name]['final_valid'].add(stamp)
            else:
                streams[name]['final_invalid_reports'] += 1
        def diagnostics(message, name=name):
            for status in message.status:
                diagnostic[name] = {v.key: v.value for v in status.values}
                reasons[name][status.message] += 1
                for key in ('model_points', 'rim_max_slice_points', 'rim_max_inliers', 'rim_max_arc',
                            'rim_radius_hypotheses', 'rim_geometry_hypotheses', 'rim_edge_hypotheses',
                            'rim_interior_rejections'):
                    if key in diagnostic[name]:
                        value = float(diagnostic[name][key])
                        peaks = sampled_peaks.setdefault(name, {})
                        peaks[key] = max(peaks.get(key, 0.), value)
        subscriptions.extend([
            node.create_subscription(MovingBucketTrack, prefix + '/target_raw', raw, 100),
            node.create_subscription(MovingBucketTrack, prefix + '/target', final, 100),
            node.create_subscription(DiagnosticArray, prefix + '/diagnostics', diagnostics, 10)])
    children, logs = [], []
    def launch(name, command):
        log = (run_dir / (name + '.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        children.append((name, process))
        return process
    def spin(duration, check_processes=True):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
            if check_processes:
                for name, process in children:
                    if process.poll() is not None:
                        raise RuntimeError(f'{name} exited ({process.returncode}); see {run_dir}')
    try:
        if args.recompute_localization:
            launch('localization', ['ros2', 'launch', 'gn10_pointcloud_localization',
                                    'localization.launch.py', 'use_sim_time:=true'])
        launch('opening', ['ros2', 'launch', 'lidar_perception_system', 'bucket_opening.launch.py',
                           'use_sim_time:=true', 'config_file:=' + str(args.config.resolve())])
        if args.baseline_executable:
            launch('baseline', [str(args.baseline_executable.resolve()), '--ros-args',
                               '-p', 'use_sim_time:=true', '-p', 'config_file:=' + str(args.baseline_config.resolve())])
            guard = Path(get_package_prefix('lidar_perception_system')) / 'lib' / 'lidar_perception_system' / 'track_freshness_guard_node'
            launch('baseline_guard', [str(guard), '--ros-args', '-p', 'use_sim_time:=true',
                                      '-p', 'config_file:=' + str(args.baseline_config.resolve())])
        spin(4.)
        player = launch('bag', ['ros2', 'bag', 'play', str(args.bag.resolve()), '--clock',
                                '--start-offset', str(args.offset - args.warmup_seconds),
                                '--disable-keyboard-controls', '--topics', *replay_topics])
        start = time.monotonic()
        # Reader startup and DDS discovery consume wall time without advancing
        # the bag. End at the requested measurement interval, not a wall timer.
        while (node.get_clock().now().nanoseconds < evaluation_end_ns
               and player.poll() is None):
            rclpy.spin_once(node, timeout_sec=.01)
            if time.monotonic() - start > args.seconds + args.warmup_seconds + 20:
                raise RuntimeError('Replay clock did not reach the evaluation end')
            for name, process in children:
                if name != 'bag' and process.poll() is not None:
                    raise RuntimeError(f'{name} exited ({process.returncode}); see {run_dir}')
        evaluation_completed = node.get_clock().now().nanoseconds >= evaluation_end_ns
        stop(player)
        # Admit results still in flight at the boundary; their measurement
        # stamps, rather than callback arrival time, determine inclusion.
        spin(max(.3, config.get('tf_wait_s', .2) + .1), check_processes=False)
        for name, process in children:
            if name != 'bag' and process.poll() is not None:
                raise RuntimeError(f'{name} exited ({process.returncode}); see {run_dir}')
        report = {'offset_s': args.offset, 'elapsed_wall_s': time.monotonic() - start,
                  'evaluation_duration_s': args.seconds, 'warmup_s': args.warmup_seconds,
                  'evaluation_completed': evaluation_completed,
                  'tf_source': 'recomputed' if args.recompute_localization else 'recorded',
                  'input_topic': config['input_topic'],
                  'diagnostics_scope': 'Lifetime snapshots, including warmup; not frame totals.',
                  'clouds_received': cloud_count[0], 'streams': {}, 'diagnostics': diagnostic,
                  'status_samples': {k: dict(v) for k, v in reasons.items()},
                  'sampled_diagnostic_peaks': sampled_peaks,
                  'accuracy': 'Not evaluated: recorded algorithm outputs are not ground truth.',
                  'logs_directory': str(run_dir)}
        for name, stream in streams.items():
            measured = list(stream['raw'].values())
            report['streams'][name] = {
                'unique_raw_results': len(measured),
                'direct_results': sum(valid and mode == 1 for valid, mode, _ in measured),
                'inferred_results': sum(valid and mode == 2 for valid, mode, _ in measured),
                'predicted_results': sum(valid and mode == 3 for valid, mode, _ in measured),
                'invalid_results': sum(not valid for valid, _, _ in measured),
                'guarded_valid_measurements': len(stream['final_valid']),
                'guarded_invalid_reports': stream['final_invalid_reports'],
                'valid_track_ids': len({track for valid, _, track in measured if valid})}
        report['opening_confirmed'] = report['streams']['opening']['direct_results'] > 0
        report['bottom'] = {'valid_measurements': len(bottom_observations),
                            'track_ids': len(set(bottom_observations.values())),
                            'accuracy_evaluated': False}
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
        if (cloud_count[0] == 0 or not streams['opening']['raw']
                or int(diagnostic.get('opening', {}).get('processed', '0')) == 0):
            raise RuntimeError('No input or opening processing results; inspect the logs')
        if not evaluation_completed:
            raise RuntimeError('The recording ended before the requested evaluation interval')
    finally:
        for _, process in reversed(children):
            stop(process)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
