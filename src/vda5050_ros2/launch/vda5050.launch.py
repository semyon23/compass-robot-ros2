"""Агент VDA 5050: ros2 launch vda5050_ros2 vda5050.launch.py robot_config:=<папка конфига робота>

Параметры — <robot_config>/vda5050.yaml. Пароль брокера — переменная окружения MQTT_PASS.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    robot_config = LaunchConfiguration('robot_config').perform(context)
    return [Node(
        package='vda5050_ros2',
        executable='vda5050_adapter',
        output='screen',
        parameters=[os.path.join(robot_config, 'vda5050.yaml'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_config', description='папка конфига робота (vda5050.yaml)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
