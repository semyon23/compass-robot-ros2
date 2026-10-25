"""Доступ к Compass снаружи робота: rosbridge (только localhost) и веб-сервер compass_web.

Можно остановить и перезапустить, не трогая ядро и робота: UI переподключится сам.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('compass_bringup'), 'config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('robot_config', description='каталог конфигов робота (образец — compass_sim/config)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('data_dir', default_value='/var/lib/robot'),

        # Веб-сервер: UI, /files, прокси /rosbridge — снаружи робота открыт только он.
        Node(
            package='compass_web',
            executable='compass_web',
            parameters=[
                PathJoinSubstitution([LaunchConfiguration('robot_config'), 'compass_web.yaml']),
                {'data_dir': LaunchConfiguration('data_dir')},
            ],
            output='screen',
        ),
        # Только нода rosbridge, без rosapi: rosbridge сам разрешает /rosapi/*, а через rosapi
        # видны все топики и сервисы робота.
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[os.path.join(config, 'rosbridge.yaml'), {'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
