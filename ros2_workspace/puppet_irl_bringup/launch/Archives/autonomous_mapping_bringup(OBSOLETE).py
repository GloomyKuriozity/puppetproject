import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler, TimerAction, IncludeLaunchDescription)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # --- Arguments you already use in your Nav2 launch ---
    map_yaml_file = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            os.getenv('HOME'),
            'maps_library',
            'test_est.yaml'
        ),
        description='Full path to map yaml file'
    )
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            os.getenv('HOME'),
            'ros2_ws',
            'src',
            'puppet_irl_bringup',
            'config',
            'nav2_params.yaml'
        ),
        description='Full path to the Nav2 parameters file'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )

    # --- slam_toolbox node (Lifecycle) ---
    slam_params_file_default = os.path.join(
        os.getenv('HOME'),
        'ros2_ws',
        'src',
        'puppet_irl_bringup',
        'config',
        'mapper_params_online_async.yaml'
    )

    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file_default,
            {
                'use_lifecycle_manager': False,
                'use_sim_time': use_sim_time,
                'max_subscriber_queue_size': 100
            }
        ],
        namespace=''
    )

    # Configure & activate slam_toolbox (same behavior as your separate launch)
    delayed_configure_slam = TimerAction(
        period=3.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_CONFIGURE
                )
            )
        ]
    )

    delayed_activate_slam = TimerAction(
        period=3.0,
        actions=[
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=slam_toolbox_node,
                    start_state='configuring',
                    goal_state='inactive',
                    entities=[
                        LogInfo(msg='[bringup] slam_toolbox configured, activating...'),
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(slam_toolbox_node),
                                transition_id=Transition.TRANSITION_ACTIVATE
                            )
                        )
                    ]
                )
            )
        ]
    )

    # --- Include your existing Nav2 launch, but only after SLAM is Active ---
    pkg_share = get_package_share_directory('puppet_irl_bringup')  # adjust if your package name differs
    nav2_launch_path = os.path.join(pkg_share, 'launch', 'nav2_launch.py')

    include_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': nav2_params_file
        }.items()
    )

    start_nav2_when_slam_active = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='activating',
            goal_state='active',
            entities=[
                LogInfo(msg='[bringup] slam_toolbox is ACTIVE -> launching Nav2 stack...'),
                include_nav2
            ]
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_map)
    ld.add_action(declare_params)
    ld.add_action(declare_use_sim_time)

    ld.add_action(slam_toolbox_node)
    ld.add_action(delayed_configure_slam)
    ld.add_action(delayed_activate_slam)
    ld.add_action(start_nav2_when_slam_active)

    return ld
