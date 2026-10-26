"""Разработка без робота: TurtleBot4 в Gazebo + Compass с конфигами робота из compass_sim/config.

ros2 launch compass_sim sim.launch.py            — с окном Gazebo и RViz
ros2 launch compass_sim sim.launch.py headless:=True
ros2 launch compass_sim sim.launch.py vda5050:=true  — и агент VDA 5050 (брокер в config/vda5050.yaml)

Карты — в ~/.local/share/compass (на роботе /var/lib/robot).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    this_dir = os.path.join(get_package_share_directory('compass_sim'), 'launch')
    bringup = os.path.join(get_package_share_directory('compass_bringup'), 'launch')
    robot_config = os.path.join(get_package_share_directory('compass_sim'), 'config')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='False', description='без окна Gazebo и RViz'),
        DeclareLaunchArgument('vda5050', default_value='false', description='запустить агент VDA 5050'),

        # Gazebo, робот, robot_state_publisher и мост odom / tf / scan / cmd_vel. Без Nav2.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(this_dir, 'tb4.launch.py')),
            launch_arguments={'headless': LaunchConfiguration('headless')}.items(),
        ),
        # В Gazebo TB4 нет BMS: заряд, который тратится в движении.
        Node(package='compass_sim', executable='fake_battery.py', parameters=[{'use_sim_time': True}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'compass.launch.py')),
            launch_arguments={
                'robot_config': robot_config,
                'use_sim_time': 'true',
                'data_dir': os.path.expanduser('~/.local/share/compass'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('vda5050_ros2'), 'launch', 'vda5050.launch.py')),
            launch_arguments={'robot_config': robot_config, 'use_sim_time': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('vda5050')),
        ),
    ])
