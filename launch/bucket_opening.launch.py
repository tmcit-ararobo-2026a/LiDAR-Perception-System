"""CPU robot-frame opening tracker with an independent final-result guard."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('lidar_perception_system'),
                          'config', 'bucket_opening.json')
    common = {'config_file': LaunchConfiguration('config_file'),
              'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config,
                              description='JSON with an explicit map-frame opponent_roi'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='lidar_perception_system', executable='bucket_opening_tracker',
             name='bucket_opening_tracker', output='screen', parameters=[common]),
        Node(package='lidar_perception_system', executable='track_freshness_guard_node',
             name='opening_freshness_guard', output='screen', parameters=[common, {
                 'raw_track_topic': '/bucket_opening_tracker/target_raw',
                 'output_topic': '/bucket_opening_tracker/target',
                 'child_frame': 'bucket_opening',
             }]),
    ])
