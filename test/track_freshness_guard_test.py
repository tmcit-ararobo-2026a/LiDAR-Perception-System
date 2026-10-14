"""Verify independent input/result/observation expiry without a CUDA device."""

import os
import subprocess
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from lidar_perception_system.msg import MovingBucketTrack


def test_track_guard_expiry():
    """Reject stale/replayed tracks and expire when sensor or processor stops."""
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = Node('track_guard_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    clouds = node.create_publisher(PointCloud2, '/livox/lidar', 10)
    raw = node.create_publisher(MovingBucketTrack, '/moving_bucket_detector/target_raw', 10)
    outputs = []
    node.create_subscription(MovingBucketTrack, '/moving_bucket_detector/target', outputs.append, 10)
    process = subprocess.Popen([os.environ['TRACK_GUARD_EXECUTABLE'], '--ros-args',
                                '-p', 'input_timeout_s:=0.1', '-p', 'track_timeout_s:=0.1',
                                '-p', 'observation_timeout_s:=0.1'],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    def send(cloud=True, track=True, frozen=None, old_observation=False):
        """Publish current synthetic inputs or a selected replay."""
        stamp = node.get_clock().now().to_msg()
        if cloud:
            message = PointCloud2()
            message.header.stamp = stamp
            message.header.frame_id = 'sensor'
            clouds.publish(message)
        if track:
            message = MovingBucketTrack()
            message.header.stamp = stamp
            message.header.frame_id = 'map'
            message.last_observation_stamp = stamp
            message.track_id = 1
            message.valid = True
            message.observation_mode = MovingBucketTrack.DIRECT
            message.confidence = 0.8
            message.position.x = 1.0
            if old_observation:
                message.observation_mode = MovingBucketTrack.PREDICTED
                message.last_observation_stamp.sec -= 1
            raw.publish(frozen or message)
            return frozen or message

    def wait_for(predicate, repeated=None):
        """Wait while dispatching isolated DDS callbacks."""
        end = time.monotonic() + 5
        while time.monotonic() < end:
            assert process.poll() is None
            if repeated:
                repeated()
            executor.spin_once(timeout_sec=0.005)
            if predicate():
                return
        raise AssertionError('track guard outcome timed out')

    try:
        wait_for(lambda: outputs and not outputs[-1].valid)
        wait_for(lambda: outputs[-1].valid, repeated=send)
        assert outputs[-1].position.x == 1.0
        original_stamp = outputs[-1].header.stamp
        wait_for(lambda: not outputs[-1].valid)
        assert outputs[-1].observation_mode == MovingBucketTrack.NONE
        wait_for(lambda: outputs[-1].valid, repeated=send)
        # Detector stops but input sensor remains alive.
        wait_for(lambda: not outputs[-1].valid, repeated=lambda: send(track=False))
        wait_for(lambda: outputs[-1].valid, repeated=send)
        # Sensor stops but stale detector activity must not permit tracking.
        wait_for(lambda: not outputs[-1].valid, repeated=lambda: send(cloud=False))
        wait_for(lambda: outputs[-1].valid, repeated=send)
        frozen = send()
        wait_for(lambda: not outputs[-1].valid,
                 repeated=lambda: send(frozen=frozen))
        # A new result timestamp cannot make an old observation fresh.
        before = len(outputs)
        send(old_observation=True)
        wait_for(lambda: len(outputs) > before and not outputs[-1].valid,
                 repeated=lambda: send(old_observation=True))
        # Explicit invalid raw output must invalidate a current target immediately.
        wait_for(lambda: outputs[-1].valid, repeated=send)
        invalid = MovingBucketTrack()
        invalid.header.stamp = original_stamp
        invalid.valid = False
        raw.publish(invalid)
        wait_for(lambda: not outputs[-1].valid)
    finally:
        process.terminate()
        try:
            process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate(timeout=5)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
