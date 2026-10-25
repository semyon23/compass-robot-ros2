"""SLAM для режима TEACH_IN. Запускает compass_core дочерним процессом, вручную не нужен.

map_file — путь к сохранённой карте без расширения (…/maps/{id}/map): достроить её,
slam_toolbox загрузит map.posegraph + map.data. Пусто — новая карта.
start_pose — поза робота на этой карте «[x, y, theta]» (робот локализован на ней); без неё
(start_at_dock:=true) slam_toolbox считает, что робот стоит там, где карту начинали.
params_file — slam_toolbox.yaml из каталога конфигов робота.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_file_name': ParameterValue(LaunchConfiguration('map_file'), value_type=str),
                # Продолжение: с позы локализации или «там, где начинали эту карту».
                'map_start_at_dock': ParameterValue(LaunchConfiguration('start_at_dock'), value_type=bool),
                'map_start_pose': LaunchConfiguration('start_pose'),
            },
        ],
        output='screen',
    )

    # slam_toolbox в Jazzy — lifecycle-нода: configure, затем activate (как online_async_launch.py).
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam,
        start_state='configuring',
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_ACTIVATE,
        ))],
    ))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('start_pose', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('start_at_dock', default_value='true'),
        DeclareLaunchArgument('params_file'),
        slam,
        configure,
        activate,
    ])
