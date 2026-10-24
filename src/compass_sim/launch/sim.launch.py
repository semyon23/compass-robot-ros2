"""Разработка без робота: TurtleBot4 в Gazebo + сервисная часть.

ros2 launch compass_sim sim.launch.py            — с окном Gazebo и RViz
ros2 launch compass_sim sim.launch.py headless:=True

Карты — в ~/.local/share/compass (на роботе /var/lib/robot).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tb4_sim = os.path.join(get_package_share_directory('nav2_minimal_tb4_sim'), 'launch')
    bringup = os.path.join(get_package_share_directory('compass_bringup'), 'launch')

    return LaunchDescription([
        # True / False с заглавной: simulation.launch.py подставляет значение в Python-выражение.
        DeclareLaunchArgument('headless', default_value='False', description='без окна Gazebo'),

        # Gazebo, робот, robot_state_publisher и мост odom / tf / scan / cmd_vel. Без Nav2.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb4_sim, 'simulation.launch.py')),
            launch_arguments={'headless': LaunchConfiguration('headless')}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'robot.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'data_dir': os.path.expanduser('~/.local/share/compass'),
            }.items(),
        ),
    ])
