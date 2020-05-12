"""Launch the rover_simulation node"""
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Start static transforms."""
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Parameters
    sim_param = {'use_sim_time': use_sim_time}

    return LaunchDescription([
        # Node(
        #   package='tf2_ros',
        #   node_executable='static_transform_publisher',
        #   node_name='odom_broadcaster',
        #   arguments=['1', '0.5', '0', '0', '0', '0', 'map', 'odom']
        # ),
        Node(
          package='tf2_ros',
          node_executable='static_transform_publisher',
          node_name='base_link_broadcaster',
          arguments=['0', '0', '0.0', '0', '0', '0', 'odom', 'base_link'],
          parameters=[sim_param]
        ),
    ])
