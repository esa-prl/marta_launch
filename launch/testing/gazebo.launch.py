"""Launch the rover_simulation node"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_helpers import get_ws_src_directory, to_urdf

from launch_ros.actions import Node

namespace_ = ''

def generate_launch_description():

    # Load Directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    marta_launch_dir = os.path.join(get_package_share_directory('marta_launch'), 'launch')
    rover_config_dir = os.path.join(get_package_share_directory('rover_config'))

    # Create urdf file from xacro and gazebo file from the package rover_config
    pkg_rover_config = get_package_share_directory('rover_config')
    xacro_model = os.path.join(pkg_rover_config, 'urdf', 'marta.xacro')
    urdf_model = to_urdf(xacro_model)

    world = LaunchConfiguration('world')

    world_1 = os.path.join(rover_config_dir, 'worlds', 'turtlebot3_houses', 'waffle.model')

    world_2 = os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world')

    world_3 = os.path.join(rover_config_dir, 'worlds', 'empty_worlds', 'world_only.model')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=[world_3, ''],
        description='SDF world file')

    # Gazebo launch
    # Starts the gzserver (handles computations) and gzclient (handles visualization)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
    )

    # Spawn rover
    spawn_rover_cmd = Node(
        package='gazebo_ros',
        node_executable='spawn_entity.py',
        node_name='spawn_entity',
        node_namespace=namespace_,
        output='screen',
        emulate_tty=True,
        arguments=['-entity',
                   'marta',
                   '-x', '-1.5', '-y', '-0.5', '-z', '1',
                   '-file', urdf_model,
                   '-reference_frame', 'world']
    )

    return LaunchDescription([
        # Launch Arguments
        declare_world_cmd,

        gazebo,
        # spawn_rover_cmd,

    ])
