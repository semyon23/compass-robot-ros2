"""TurtleBot4 в Gazebo (склад depot): сервер Gazebo, робот, мост ROS ↔ Gazebo, robot_state_publisher.

Мир — своя копия depot.sdf (worlds/) с шагом физики 5 мс вместо 3 мс: меньше нагрузка на весь стек.

Копия nav2_minimal_tb4_sim/simulation.launch.py без одной гонки: там xacro пишет мир в /tmp
параллельно со стартом gz sim, и на нагруженной машине Gazebo падает с «Unable to find or download
file». Здесь gz sim стартует, когда xacro закончил. RViz — только с окном (headless:=False).
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = get_package_share_directory('nav2_minimal_tb4_sim')
    desc_dir = get_package_share_directory('nav2_minimal_tb4_description')
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    headless = LaunchConfiguration('headless')
    robot_sdf = os.path.join(desc_dir, 'urdf', 'standard', 'turtlebot4.urdf.xacro')

    world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    world = os.path.join(get_package_share_directory('compass_sim'), 'worlds', 'depot.sdf')
    world_xacro = ExecuteProcess(cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world])
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch), launch_arguments={'gz_args': ['-r -s ', world_sdf]}.items())

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='False', description='без окна Gazebo и RViz'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'worlds')),

        world_xacro,
        RegisterEventHandler(OnProcessExit(target_action=world_xacro, on_exit=[gazebo_server])),
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda _: os.path.exists(world_sdf) and os.remove(world_sdf))])),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch), launch_arguments={'gz_args': '-v4 -g '}.items(),
            condition=UnlessCondition(headless)),

        # Робот в мире и мост odom / tf / scan / cmd_vel. Поза — как в nav2_minimal_tb4_sim.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_dir, 'launch', 'spawn_tb4.launch.py')),
            launch_arguments={'use_sim_time': 'True', 'robot_sdf': robot_sdf,
                              'x_pose': '-8.00', 'y_pose': '0.00'}.items()),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro', ' ', robot_sdf])}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', os.path.join(desc_dir, 'rviz', 'config.rviz')],
            parameters=[{'use_sim_time': True}], remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            condition=UnlessCondition(headless)),
    ])
