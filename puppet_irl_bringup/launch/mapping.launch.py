from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    puppet_dir = get_package_share_directory('puppet_irl_bringup')
    explore_dir = get_package_share_directory('explore_lite_IRL')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([puppet_dir, 'launch', 'slam_toolbox.launch.py'])
        ]),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([puppet_dir, 'launch', 'nav2_launch.py'])
        ]),
    )

    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([explore_dir, 'launch', 'explore.launch.py'])
        ]),
    )

    return LaunchDescription([
        slam_launch,
        TimerAction(period=5.0, actions=[nav2_launch]),
        TimerAction(period=15.0, actions=[explore_launch])
    ])
