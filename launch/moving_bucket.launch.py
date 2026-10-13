import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lidar_perception_system')
    config = os.path.join(share, 'config', 'moving_bucket.json')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='lidar_perception_system',
            executable='moving_bucket_detector',
            name='moving_bucket_detector',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
