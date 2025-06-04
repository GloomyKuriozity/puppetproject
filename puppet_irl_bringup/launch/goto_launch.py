from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    bringup_dir = os.path.join(
        os.getenv('HOME'),
        'ros2_ws',
        'src',
        'puppet_irl_bringup',
        'launch'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                os.getenv('HOME'),
                'maps_library',
                'test_est.yaml'
            ),
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                os.getenv('HOME'),
                'ros2_ws',
                'src',
                'puppet_irl_bringup',
                'config',
                'nav2_params.yaml'
            ),
            description='Full path to the ROS2 params file'
        ),

        # Delay then start Lifecycle Manager for map_server
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_map',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'autostart': True,
                        'node_names': ['map_server']
                    }]
                )
            ]
        ),

        # Delay again, then launch Nav2 system
        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_dir, 'nav2_launch.py')
                    ),
                    launch_arguments={
                        'map': map_yaml_file,
                        'params_file': params_file
                    }.items()
                )
            ]
        )
    ])
