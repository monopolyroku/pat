"""Synthetic plant + state_estimator over CycloneDDS.

    ros2 launch pat_estimator estimator_sim.launch.py \
        fusion_mode:=replay|apply_at_arrival \
        plant_params:=<yaml> estimator_params:=<yaml>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory('pat_sim')
    est_share = get_package_share_directory('pat_estimator')

    return LaunchDescription([
        DeclareLaunchArgument(
            'plant_params',
            default_value=os.path.join(sim_share, 'config', 'plant_default.yaml')),
        DeclareLaunchArgument(
            'estimator_params',
            default_value=os.path.join(est_share, 'config', 'estimator_default.yaml')),
        DeclareLaunchArgument('fusion_mode', default_value='replay',
                              choices=['replay', 'apply_at_arrival']),
        DeclareLaunchArgument('fast_thread_priority', default_value='0'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable(
            'CYCLONEDDS_URI',
            'file://' + os.path.join(sim_share, 'config', 'cyclonedds_localhost.xml')),
        Node(
            package='pat_sim', executable='synthetic_plant', name='synthetic_plant',
            output='screen',
            parameters=[LaunchConfiguration('plant_params')]),
        Node(
            package='pat_estimator', executable='state_estimator', name='state_estimator',
            output='screen',
            parameters=[
                LaunchConfiguration('estimator_params'),
                {'fusion_mode': LaunchConfiguration('fusion_mode'),
                 'fast_thread_priority': ParameterValue(
                     LaunchConfiguration('fast_thread_priority'), value_type=int)},
            ]),
    ])
