"""Ядро Compass: compass_core, twist_mux.

SLAM и навигацию (Nav2) compass_core запускает сам — slam.launch.py и nav.launch.py с параметрами
из каталога конфигов робота. Драйверы робота (база, лидар, ros2_control, robot_state_publisher)
запускаются отдельно и дают TF odom → base_link, топики scan и odom, принимают cmd_vel_topic.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def twist_mux(context, config):
    """twist_mux: тип скорости (Twist / TwistStamped) — как у базы, флаг cmd_vel_stamped в
    compass_core.yaml робота: один флаг на всех, иначе джойстик и Nav2 разойдутся с twist_mux."""
    robot_config = LaunchConfiguration('robot_config').perform(context)
    with open(os.path.join(robot_config, 'compass_core.yaml')) as f:
        params = yaml.safe_load(f)['compass_core']['ros__parameters']
    return [Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[
            os.path.join(config, 'twist_mux.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'use_stamped': bool(params.get('cmd_vel_stamped', False))},
        ],
        remappings=[('cmd_vel_out', LaunchConfiguration('cmd_vel_topic'))],
        output='screen',
    )]


def generate_launch_description():
    config = os.path.join(get_package_share_directory('compass_bringup'), 'config')
    robot_config = LaunchConfiguration('robot_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    data_dir = LaunchConfiguration('data_dir')

    return LaunchDescription([
        DeclareLaunchArgument('robot_config', description='каталог конфигов робота (образец — compass_sim/config)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Карты, настройки, журнал, события (ROS_INTERFACES.md §5).
        DeclareLaunchArgument('data_dir', default_value='/var/lib/robot'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel', description='куда база принимает скорость'),

        Node(
            package='compass_core',
            executable='compass_core',
            parameters=[
                PathJoinSubstitution([robot_config, 'compass_core.yaml']),
                {
                    'use_sim_time': use_sim_time,
                    'data_dir': data_dir,
                    'config_schema': os.path.join(config, 'robot_schema.yaml'),
                    # Параметры стеков, которые compass_core запускает сам.
                    'nav_params': PathJoinSubstitution([robot_config, 'nav2.yaml']),
                    'slam_params': PathJoinSubstitution([robot_config, 'slam_toolbox.yaml']),
                },
            ],
            output='screen',
        ),
        # Кто управляет скоростью: джойстик Compass приоритетнее Nav2 (config/twist_mux.yaml).
        OpaqueFunction(function=twist_mux, kwargs={'config': config}),
    ])
