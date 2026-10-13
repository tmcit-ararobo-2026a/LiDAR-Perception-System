#!/usr/bin/env python3
"""Replay a rosbag segment and measure the CUDA detector. Reference is not truth."""

import argparse
import bisect
import json
import os
import secrets
import sqlite3
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PointStamped
from lidar_perception_system.msg import MovingBucketTrack
from diagnostic_msgs.msg import DiagnosticArray
from visualization_msgs.msg import Marker, MarkerArray
from tf2_msgs.msg import TFMessage


class Monitor(Node):
    def __init__(self):
        super().__init__("cuda_bag_benchmark")
        self.counts = {"received": 0, "valid": 0, "direct": 0, "predicted": 0,
                       "invalid": 0, "reference": 0,
                       "marker_messages": 0, "marker_target_add": 0,
                       "marker_target_delete": 0, "marker_status": 0}
        self.outputs = []
        self.target_events = []
        self.target_trace = []
        self.references = []
        self.last_diagnostic = {}
        self.diagnostic_trace = []
        self.static_tf_pub = self.create_publisher(TFMessage, "/tf_static",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(MovingBucketTrack, "/moving_bucket_detector/target",
                                 self.on_target, 100)
        self.create_subscription(PointStamped, "/opponent_robot/bucket_target",
                                 self.on_reference, 100)
        self.create_subscription(DiagnosticArray,
                                 "/moving_bucket_detector/diagnostics",
                                 self.on_diagnostic, 10)
        self.create_subscription(MarkerArray, "/moving_bucket_detector/markers",
                                 self.on_markers, 10)

    @staticmethod
    def stamp(value):
        return value.sec + value.nanosec / 1e9

    def on_target(self, msg):
        self.counts["received"] += 1
        self.target_events.append((self.stamp(msg.header.stamp), msg.valid,
                                   msg.observation_mode, msg.track_id))
        self.target_trace.append((self.stamp(msg.header.stamp), msg.valid,
                                  msg.observation_mode, msg.track_id,
                                  msg.position.x, msg.position.y, msg.position.z))
        if msg.valid:
            self.counts["valid"] += 1
            if msg.observation_mode == MovingBucketTrack.DIRECT:
                self.counts["direct"] += 1
            elif msg.observation_mode == MovingBucketTrack.PREDICTED:
                self.counts["predicted"] += 1
            self.outputs.append((self.stamp(msg.header.stamp), msg.position.x,
                                 msg.position.y, msg.position.z))
        else:
            self.counts["invalid"] += 1

    def on_reference(self, msg):
        self.counts["reference"] += 1
        self.references.append((self.stamp(msg.header.stamp), msg.point.x,
                                msg.point.y, msg.point.z))

    def on_markers(self, msg):
        self.counts["marker_messages"] += 1
        for marker in msg.markers:
            if marker.id == 0 and marker.action == Marker.ADD:
                self.counts["marker_target_add"] += 1
            elif marker.id == 0 and marker.action == Marker.DELETE:
                self.counts["marker_target_delete"] += 1
            elif marker.id == 4 and marker.action == Marker.ADD:
                self.counts["marker_status"] += 1

    def on_diagnostic(self, msg):
        for status in msg.status:
            if status.name == "moving_bucket_detector":
                self.last_diagnostic = {value.key: value.value for value in status.values}
                self.diagnostic_trace.append((self.stamp(msg.header.stamp),
                                              self.last_diagnostic.copy()))


def stop(group):
    if group.poll() is None:
        os.killpg(group.pid, signal.SIGINT)
        try:
            group.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(group.pid, signal.SIGTERM)
            group.wait(timeout=5)


def compare(outputs, references):
    references.sort()
    times = [row[0] for row in references]
    errors = []
    for sample in outputs:
        index = bisect.bisect_left(times, sample[0])
        neighbors = references[max(index - 1, 0):min(index + 1, len(references))]
        if not neighbors:
            continue
        near = min(neighbors, key=lambda row: abs(row[0] - sample[0]))
        if abs(near[0] - sample[0]) <= 0.3:
            errors.append(((sample[1] - near[1]) ** 2 +
                           (sample[2] - near[2]) ** 2) ** 0.5)
    errors.sort()
    return {"matched": len(errors),
            "xy_delta_median_m": errors[len(errors) // 2] if errors else None,
            "xy_delta_p95_m": errors[int((len(errors) - 1) * 0.95)] if errors else None,
            "within_1m": sum(error <= 1.0 for error in errors)}


def continuity(events):
    if not events:
        return {"valid_fraction": None, "longest_invalid_run_s": None,
                "track_ids": 0, "track_switches": 0}
    valid = sum(event[1] for event in events)
    ids = [event[3] for event in events if event[1]]
    longest = 0.0
    invalid_start = None
    for stamp, is_valid, _, _ in events:
        if not is_valid and invalid_start is None:
            invalid_start = stamp
        elif is_valid and invalid_start is not None:
            longest = max(longest, stamp - invalid_start)
            invalid_start = None
    if invalid_start is not None:
        longest = max(longest, events[-1][0] - invalid_start)
    return {"valid_fraction": valid / len(events),
            "longest_invalid_run_s": longest,
            "track_ids": len(set(ids)),
            "track_switches": sum(a != b for a, b in zip(ids, ids[1:]))}


def detector_pid(launch_pid, domain_id):
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            args = (entry / "cmdline").read_bytes().split(b"\0")
        except (OSError, PermissionError):
            continue
        if not any(Path(arg.decode(errors="replace")).name == "moving_bucket_detector"
                   for arg in args if arg):
            continue
        try:
            parent_line = next(line for line in (entry / "status").read_text().splitlines()
                               if line.startswith("PPid:"))
            environment = (entry / "environ").read_bytes().split(b"\0")
        except (OSError, PermissionError, StopIteration):
            continue
        if int(parent_line.split()[1]) != launch_pid:
            continue
        if ("ROS_DOMAIN_ID=" + domain_id).encode() not in environment:
            continue
        return int(entry.name)
    return None


def process_usage(pid):
    fields = Path(f"/proc/{pid}/stat").read_text().split()
    ticks = int(fields[13]) + int(fields[14])
    lines = Path(f"/proc/{pid}/status").read_text().splitlines()
    rss = next(int(line.split()[1]) for line in lines if line.startswith("VmRSS:")) / 1024
    return ticks, rss


def gpu_memory(pid):
    result = subprocess.run(
        ["nvidia-smi", "--query-compute-apps=pid,used_gpu_memory",
         "--format=csv,noheader,nounits"],
        capture_output=True, text=True, timeout=2, check=False)
    if result.returncode != 0:
        return None
    for line in result.stdout.splitlines():
        parts = [part.strip() for part in line.split(",")]
        if len(parts) == 2 and parts[0] == str(pid):
            return float(parts[1])
    return 0.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--offset", type=float, default=65)
    parser.add_argument("--seconds", type=float, default=50)
    parser.add_argument("--output", type=Path, default=Path("/tmp/lps_cuda_benchmark.json"))
    parser.add_argument("--include-traces", action="store_true",
                        help="Save complete target/reference traces to the local output file")
    args = parser.parse_args()
    env = os.environ.copy()
    if not env.get("ROS_DOMAIN_ID"):
        env["ROS_DOMAIN_ID"] = str(20 + secrets.randbelow(180))
        os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    prefix = f"/tmp/lps_cuda_{os.getpid()}"
    env["ROS_LOG_DIR"] = prefix + "_ros_logs"
    env["ROS2CLI_DISABLE_DAEMON"] = "1"
    Path(env["ROS_LOG_DIR"]).mkdir(exist_ok=True)
    with open(prefix + "_launch.log", "w") as launch_log, open(
            prefix + "_bag.log", "w") as bag_log:
        launch = subprocess.Popen(
            ["ros2", "launch", "lidar_perception_system", "moving_bucket.launch.py"],
            stdout=launch_log, stderr=subprocess.STDOUT, env=env,
            start_new_session=True)
        rclpy.init()
        monitor = Monitor()
        database = args.bag if args.bag.is_file() else next(args.bag.glob("*.db3"))
        with sqlite3.connect("file:" + str(database) + "?mode=ro", uri=True) as connection:
            static_rows = connection.execute(
                "SELECT data FROM messages WHERE topic_id="
                "(SELECT id FROM topics WHERE name='/tf_static')").fetchall()
        static_message = TFMessage()
        for (blob,) in static_rows:
            static_message.transforms.extend(deserialize_message(blob, TFMessage).transforms)
        monitor.static_tf_pub.publish(static_message)
        bag = None
        try:
            warmup = time.monotonic() + 3.0
            while time.monotonic() < warmup:
                rclpy.spin_once(monitor, timeout_sec=0.05)
            bag = subprocess.Popen(
                ["ros2", "bag", "play", str(args.bag), "--start-offset",
                 str(args.offset), "--topics", "/livox/lidar", "/tf", "/tf_static",
                 "/opponent_robot/bucket_target"],
                stdout=bag_log, stderr=subprocess.STDOUT, env=env,
                start_new_session=True)
            end = time.monotonic() + args.seconds
            pid = detector_pid(launch.pid, env["ROS_DOMAIN_ID"])
            if pid is None:
                raise RuntimeError("Benchmark detector process not found in ROS_DOMAIN_ID")
            resource_samples = []
            last_ticks = None
            last_time = None
            next_sample = time.monotonic() + 1.0
            while time.monotonic() < end and bag.poll() is None:
                rclpy.spin_once(monitor, timeout_sec=0.05)
                now = time.monotonic()
                if pid is not None and now >= next_sample:
                    try:
                        ticks, rss = process_usage(pid)
                        cpu = (ticks - last_ticks) / os.sysconf("SC_CLK_TCK") / (now - last_time) * 100 if last_ticks is not None else None
                        resource_samples.append({"cpu_one_core_percent": cpu, "rss_mib": rss,
                                                 "gpu_memory_mib": gpu_memory(pid)})
                        last_ticks, last_time = ticks, now
                    except (OSError, PermissionError):
                        pass
                    next_sample += 1.0
            cpu_values = [row["cpu_one_core_percent"] for row in resource_samples if row["cpu_one_core_percent"] is not None]
            result = {"offset_s": args.offset, "duration_s": args.seconds,
                      "resources": {"samples": len(resource_samples),
                                    "cpu_mean": sum(cpu_values) / len(cpu_values) if cpu_values else None,
                                    "cpu_max": max(cpu_values) if cpu_values else None,
                                    "rss_max_mib": max((row["rss_mib"] for row in resource_samples), default=None),
                                    "gpu_memory_max_mib": max((row["gpu_memory_mib"] for row in resource_samples if row["gpu_memory_mib"] is not None), default=None)},
                      "counts": monitor.counts,
                      "continuity": continuity(monitor.target_events),
                      "diagnostics": monitor.last_diagnostic,
                      "reference_comparison_not_ground_truth": compare(
                          monitor.outputs, monitor.references)}
            if args.include_traces:
                result["target_trace"] = monitor.target_trace
                result["reference_trace"] = monitor.references
                result["diagnostic_trace"] = monitor.diagnostic_trace
            args.output.write_text(json.dumps(result, indent=2) + "\n")
            summary = {key: value for key, value in result.items()
                       if key not in ("target_trace", "reference_trace", "diagnostic_trace")}
            print(json.dumps(summary, indent=2))
        finally:
            if bag is not None:
                stop(bag)
            stop(launch)
            monitor.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
