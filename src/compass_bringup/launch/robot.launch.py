"""Сервисная часть робота: compass_core, twist_mux, task_orchestrator, rosbridge, compass_web.

SLAM и навигацию (Nav2) compass_core запускает сам — slam.launch.py и nav.launch.py.

На роботе запускается рядом с его драйверами; в разработке — из sim.launch.py.
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
    data_dir = LaunchConfiguration('data_dir')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Карты и загрузки (ROS_INTERFACES.md §5); compass_web отдаёт отсюда /files/.
        DeclareLaunchArgument('data_dir', default_value='/var/lib/robot'),

        Node(
            package='compass_core',
            executable='compass_core',
            parameters=[
                os.path.join(config, 'compass_core.yaml'),
                {
                    'use_sim_time': use_sim_time,
                    'data_dir': data_dir,
                    'config_schema': os.path.join(config, 'robot_schema.yaml'),
                    'tasks_template': os.path.join(config, 'tasks.yaml'),
                },
            ],
            output='screen',
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            parameters=[os.path.join(config, 'twist_mux.yaml'), {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel_out', 'cmd_vel')],  # выход — в базу робота
            output='screen',
        ),
        # Исполнитель задач: сервисные цели Compass (ROS_INTERFACES.md §4). История — в SQLite.
        Node(
            package='task_orchestrator_core',
            executable='task_orchestrator_node',
            name='task_orchestrator',
            parameters=[{
                'use_sim_time': use_sim_time,
                # Собирает compass_core из config/tasks.yaml и порога батареи (Настройки → Робот).
                'tasks_config_path': PathJoinSubstitution([data_dir, 'config', 'tasks.yaml']),
                'storage.enabled': True,
                'storage.sqlite_path': PathJoinSubstitution([data_dir, 'tasks.db']),
            }],
            output='screen',
        ),
        # Веб-сервер Compass: UI, /files, прокси /rosbridge — снаружи робота открыт только он.
        Node(
            package='compass_web',
            executable='compass_web',
            parameters=[os.path.join(config, 'compass_web.yaml'), {'data_dir': data_dir}],
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
