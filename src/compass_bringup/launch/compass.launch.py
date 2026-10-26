"""Compass целиком: ядро (core.launch.py) + доступ снаружи (web.launch.py).

  ros2 launch compass_bringup compass.launch.py robot_config:=<каталог конфигов робота>

Каталог конфигов робота: compass_core.yaml, nav2.yaml, slam_toolbox.yaml, compass_web.yaml
(образец для TurtleBot4 — compass_sim/config). Драйверы робота запускаются отдельно.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch = os.path.join(get_package_share_directory('compass_bringup'), 'launch')
    names = ['robot_config', 'use_sim_time', 'data_dir']
    args = {name: LaunchConfiguration(name) for name in names}

    return LaunchDescription([
        DeclareLaunchArgument('robot_config', description='каталог конфигов робота (образец — compass_sim/config)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('data_dir', default_value='/var/lib/robot'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel', description='куда база принимает скорость'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch, 'core.launch.py')),
            launch_arguments={**args, 'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic')}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch, 'web.launch.py')),
            launch_arguments={'robot_config': args['robot_config'], 'data_dir': args['data_dir']}.items(),
        ),
    ])
