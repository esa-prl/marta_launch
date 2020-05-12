"""Launch the rover_simulation node"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_helpers import get_ws_src_directory, to_urdf

from launch_ros.actions import Node

namespace_ = ''

# Get package src path based on a package name. Make sure the package is installed from source.
ros2_ws_src = get_ws_src_directory('gamepad_parser')


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Gazebo launch
    # Starts the gzserver (handles computations) and gzclient (handles visualization)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # Create urdf file from xacro and gazebo file from the package rover_config
    pkg_rover_config = get_package_share_directory('rover_config')
    xacro_model = os.path.join(pkg_rover_config, 'urdf', 'marta.xacro')
    urdf_model = to_urdf(xacro_model)

    # Spawn rover
    spawn_rover = Node(
        package='gazebo_ros',
        node_executable='spawn_entity.py',
        node_name='spawn_entity',
        node_namespace=namespace_,
        output='screen',
        emulate_tty=True,
        arguments=['-entity',
                   'marta',
                   '-x', '0', '-y', '0', '-z', '1',
                   '-file', urdf_model,
                   '-reference_frame', 'world']
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'namespace', default_value=namespace_,
            description='Top-level namespace'),
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(
                pkg_gazebo_ros, 'worlds', 'empty.world'), ''],
            description='SDF world file'
        ),
        gazebo,
        spawn_rover,

    ])
