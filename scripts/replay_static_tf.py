#!/usr/bin/env python3
"""Publish a bag's static TF for playback started after its original TF message."""

import argparse
import sqlite3
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def databases(path):
    if path.is_file() and path.suffix == '.db3':
        return [path]
    if path.is_dir():
        files = sorted(path.glob('*.db3'))
        if files:
            return files
    raise ValueError(f'No rosbag2 .db3 file at {path}')


def read_static_tf(path):
    # Repeated /tf_static messages may contain the same child frame. Publish
    # one transform per child so tf2 does not receive conflicting duplicates.
    transforms = {}
    for database in databases(path):
        with sqlite3.connect('file:' + str(database) + '?mode=ro', uri=True) as connection:
            rows = connection.execute(
                "SELECT data FROM messages WHERE topic_id="
                "(SELECT id FROM topics WHERE name='/tf_static') ORDER BY timestamp")
            for (blob,) in rows:
                for transform in deserialize_message(blob, TFMessage).transforms:
                    transforms[transform.child_frame_id] = transform
    if not transforms:
        raise ValueError(f'No /tf_static transforms found in {path}')
    message = TFMessage()
    message.transforms = list(transforms.values())
    return message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    args, ros_args = parser.parse_known_args()
    message = read_static_tf(args.bag)
    rclpy.init(args=ros_args)
    node = Node('bag_static_tf_replayer')
    publisher = node.create_publisher(
        TFMessage, '/tf_static',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE))
    publisher.publish(message)
    node.get_logger().info(f'Published {len(message.transforms)} static TF transforms')
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
