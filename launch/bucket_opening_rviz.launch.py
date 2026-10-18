"""Inspect selected robot, motion-compensated points and guarded opening results."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


def replay_static_tf(context):
    path = LaunchConfiguration('bag_path').perform(context)
    if not path:
        return []
    return [Node(package='lidar_perception_system', executable='replay_static_tf.py',
                 name='opening_static_tf_replayer', output='screen', arguments=[path])]


def generate_launch_description():
    share = get_package_share_directory('lidar_perception_system')
    config_home = os.environ.get('XDG_CONFIG_HOME') or os.path.expanduser('~/.config')
    saved = os.path.join(config_home, 'lidar_perception_system', 'bucket_opening.json')
    config = saved if os.path.isfile(saved) else os.path.join(share, 'config', 'bucket_opening.json')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('bag_path', default_value=''),
        OpaqueFunction(function=replay_static_tf),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share, 'launch', 'bucket_opening.launch.py')), launch_arguments={
                'config_file': LaunchConfiguration('config_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()),
        Node(package='rviz2', executable='rviz2', name='opening_rviz',
             arguments=['-d', os.path.join(share, 'rviz', 'bucket_opening.rviz')],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
