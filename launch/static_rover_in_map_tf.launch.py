"""Launch the rover_simulation node"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_helpers import add_namespace_to_yaml, get_ws_src_directory, to_urdf

from launch_ros.actions import Node

namespace_ = 'marta'

def generate_launch_description():

    return LaunchDescription([
        Node(
          package='tf2_ros',
          node_executable='static_transform_publisher',
          node_name='odom_broadcaster',
          arguments=['1', '0.5', '0', '0', '0', '0', 'map', 'odom']
        ),
        Node(
          package='tf2_ros',
          node_executable='static_transform_publisher',
          node_name='base_link_broadcaster',
          arguments=['0', '0', '0.2', '0', '0', '0', 'odom', 'base_link']
        ),
    ])
