from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the lidar_publisher node
    robot_control_node = Node(
        package='robot_control',  # Replace with your package name
        executable='robot_control',  # Replace with the actual node executable name
        name='robot_control',
        output='screen',
        parameters=[{'use_sim_time': False}]  # Adjust as needed
    )

    # Create and return the LaunchDescription
    ld = LaunchDescription()
    ld.add_action(robot_control_node)

    return ld
