"""Локализация и навигация на карте: map_server + AMCL сразу, Nav2 — после начальной позы.

Запускает compass_core дочерним процессом при активации карты, вручную не нужен.
map — путь к map.yaml активной карты, params_file — nav2.yaml из каталога конфигов робота.

Все ноды Nav2 — в одном процессе (composition, как в bringup_launch.py Nav2): один участник DDS
вместо ~15, сообщения между нодами без копирования, /clock и TF обрабатываются в одном процессе.
Заметно меньше нагрузка на процессор. Имена нод и сервисов те же.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2 = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    common = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'params_file': LaunchConfiguration('params_file'),
        'autostart': 'true',
        'use_composition': 'True',
        'container_name': 'nav2_container',
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('params_file'),

        # Контейнер, в который localization_launch.py и navigation_launch.py загружают ноды.
        # isolated — у каждой ноды свой executor: тяжёлый контроллер не задерживает остальных.
        Node(
            package='rclcpp_components',
            executable='component_container_isolated',
            name='nav2_container',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),

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
