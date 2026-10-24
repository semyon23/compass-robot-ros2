"""Локализация и навигация на карте: map_server + AMCL сразу, Nav2 — после начальной позы.

Запускает compass_core дочерним процессом при активации карты, вручную не нужен.
map — путь к map.yaml активной карты.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav2 = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    params = os.path.join(get_package_share_directory('compass_bringup'), 'config', 'nav2.yaml')
    common = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'params_file': params,
        'autostart': 'true',
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'localization_launch.py')),
            launch_arguments={**common, 'map': LaunchConfiguration('map')}.items(),
        ),
        # Навигацию поднимает compass_core (manage_nodes STARTUP), когда у AMCL есть поза:
        # без TF map→odom костмапы не активируются и lifecycle_manager бросает запуск.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'navigation_launch.py')),
            launch_arguments={**common, 'autostart': 'false'}.items(),
        ),
    ])
