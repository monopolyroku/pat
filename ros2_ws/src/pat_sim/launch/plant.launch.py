"""Launch the synthetic plant alone.

Sets RMW_IMPLEMENTATION and CYCLONEDDS_URI explicitly (TD-9) so a shell with a
different default RMW cannot produce a silent cross-vendor mismatch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('pat_sim')
    default_params = os.path.join(share, 'config', 'plant_default.yaml')
    cyclone_cfg = os.path.join(share, 'config', 'cyclonedds_localhost.xml')

    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=default_params,
                              description='Plant parameter file'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', 'file://' + cyclone_cfg),
        Node(
            package='pat_sim',
            executable='synthetic_plant',
            name='synthetic_plant',
            output='screen',
            parameters=[LaunchConfiguration('params')],
        ),
    ])
