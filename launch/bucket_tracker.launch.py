"""CPU robot-frame opening tracker with an independent final-result guard."""
import os
import json
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def check_roi(name, roi):
    keys = ('min_x', 'max_x', 'min_y', 'max_y')
    if (not isinstance(roi, dict) or any(
            type(roi.get(key)) not in (int, float) or
            not math.isfinite(roi[key]) or abs(roi[key]) > 100 for key in keys)
            or roi['min_x'] >= roi['max_x'] or roi['min_y'] >= roi['max_y']):
        raise ValueError(f'{name} needs finite min_x/max_x/min_y/max_y bounds')


def check_config(context):
    path = LaunchConfiguration('config_file').perform(context)
    start_zone = LaunchConfiguration('start_zone').perform(context)
    if start_zone not in ('', 'red', 'blue'):
        raise RuntimeError(f"Invalid start_zone '{start_zone}': use red, blue or leave it unset.")
    try:
        with open(path, encoding='utf-8') as stream:
            config = json.load(stream)
        if not isinstance(config, dict) or config.get('schema_version') != 1:
            raise ValueError('schema_version must be 1 in a JSON object')
        check_roi('opponent_roi', config.get('opponent_roi'))
        if start_zone:
            zones = config.get('start_zone_opponent_roi')
            check_roi(f'start_zone_opponent_roi.{start_zone}',
                      zones.get(start_zone) if isinstance(zones, dict) else None)
    except (OSError, ValueError, TypeError) as error:
        raise RuntimeError(
            f'Invalid bucket config {path}: {error}. '
            'Fix config/bucket_tracker.json and rebuild the package, or set '
            'config_file:=/absolute/path/to/config.json.') from error
    return [LogInfo(msg=f'Bucket detection config: {path}')]


def tracker_exited(event, context):
    if event.returncode != 0 and not context.is_shutdown:
        raise RuntimeError(f'Bucket tracker stopped with code {event.returncode}; '
                           'closing this launch. Check its error message.')
    return []


def generate_launch_description():
    config = os.path.join(get_package_share_directory('lidar_perception_system'),
                          'config', 'bucket_tracker.json')
    common = {'config_file': LaunchConfiguration('config_file'),
              'use_sim_time': LaunchConfiguration('use_sim_time')}
    tracker = Node(package='lidar_perception_system', executable='bucket_tracker',
                   name='bucket_tracker', output='screen', parameters=[common, {
                       'start_zone': ParameterValue(LaunchConfiguration('start_zone'),
                                                    value_type=str),
                   }])
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config,
                              description='JSON with an explicit map-frame opponent_roi'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_zone', default_value='',
                              description='Own start zone: red or blue. Unset searches both fields.'),
        OpaqueFunction(function=check_config),
        RegisterEventHandler(OnProcessExit(target_action=tracker, on_exit=tracker_exited)),
        tracker,
        Node(package='lidar_perception_system', executable='track_freshness_guard_node',
             name='opening_freshness_guard', output='screen', parameters=[common, {
                 'raw_track_topic': '/bucket_opening_tracker/target_raw',
                 'output_topic': '/bucket_opening_tracker/target',
                 'child_frame': 'bucket_opening',
             }]),
    ])
