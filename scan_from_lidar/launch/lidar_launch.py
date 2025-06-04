from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the lidar_publisher node
    lidar_publisher_node = Node(
        package='scan_from_lidar',  # Replace with your package name
        executable='scan_from_lidar',  # Replace with the actual node executable name
        name='scan_from_lidar',
        output='screen',
        parameters=[{'use_sim_time': False}]  # Adjust as needed
    )

    # Define the static transform broadcaster node
    static_tf_broadcaster_node = Node(
        package='scan_from_lidar',  # Replace with your package name
        executable='static_tf2_broadcaster',  # Replace with the actual node executable name
        name='static_tf2_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': False}]  # Adjust as needed
    )

    # Create and return the LaunchDescription
    ld = LaunchDescription()
    ld.add_action(lidar_publisher_node)
    ld.add_action(static_tf_broadcaster_node)

    return ld
