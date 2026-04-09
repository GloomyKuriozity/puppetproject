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
    
    return LaunchDescription([
    ])
