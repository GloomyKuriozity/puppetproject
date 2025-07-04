import os
import getpass
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    CURRENT_USER = getpass.getuser()
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_params_file = os.path.join(
        os.getenv('HOME'),
        'ros2_ws',
        'src',
        'puppet_irl_bringup',
        'config',
        'mapper_params_online_async.yaml'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
          slam_params_file,
          {
            'use_lifecycle_manager': use_lifecycle_manager,
            'use_sim_time': use_sim_time
          }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )

    delayed_configure_event = TimerAction(
        period=3.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_CONFIGURE
                ),
                condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
            )
        ]
    )

    delayed_activate_event = TimerAction(
        period=3.0,
        actions=[
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=start_async_slam_toolbox_node,
                    start_state="configuring",
                    goal_state="inactive",
                    entities=[
                        LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                        EmitEvent(event=ChangeState(
                            lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                            transition_id=Transition.TRANSITION_ACTIVATE
                        ))
                    ]
                ),
                condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))))
        ]
    )


    ld = LaunchDescription()

    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(delayed_configure_event)
    ld.add_action(delayed_activate_event)

    return ld

#slam_params_file = "/home/kuriozity/ros2_ws/src/puppet_v2_pkg/config/mapper_params_online_async.yaml"
