"""Launch the rover_simulation node"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_helpers import get_ws_src_directory, add_namespace_to_yaml, to_urdf

namespace_ = 'marta'

# Get package src path based on a package name. Make sure the package is installed from source.
ros2_ws_src = get_ws_src_directory('gamepad_parser')


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Gazebo launch
    # Starts the gzserver (handles computations) and gzclient (handles visualization)
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
    #     )
    # )

    # Individual Parameter files
    gamepad_parser_config = os.path.join(
        ros2_ws_src, 'gamepad_parser', 'config', 'gamepad_parser.yaml')
    locomotion_manager_config = os.path.join(
        ros2_ws_src, 'locomotion_manager', 'config', 'locomotion_manager.yaml')
    simple_rover_locomotion_config = os.path.join(
        ros2_ws_src, 'simple_rover_locomotion', 'config', 'robot_poses.yaml')
    stop_mode_config = os.path.join(
        ros2_ws_src, 'locomotion_mode', 'config', 'stop_mode.yaml')

    # Add namespace to the yaml file
    gamepad_parser_config_ns = add_namespace_to_yaml(
        namespace_, gamepad_parser_config)
    locomotion_manager_config_ns = add_namespace_to_yaml(
        namespace_, locomotion_manager_config)
    simple_rover_locomotion_config_ns = add_namespace_to_yaml(
        namespace_, simple_rover_locomotion_config)
    stop_mode_config_ns = add_namespace_to_yaml(
        namespace_, stop_mode_config)

    # Create urdf file from xacro and gazebo file from the package rover_config
    pkg_rover_config = get_package_share_directory('rover_config')
    xacro_model = os.path.join(pkg_rover_config, 'urdf', 'marta.xacro')
    urdf_model = to_urdf(xacro_model)
    urdf_params = {'urdf_model_path': urdf_model}

    rviz_model = os.path.join(pkg_rover_config, 'rviz', 'rover_gazebo.rviz')

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

    # Launch robot_state_publisher to publish the robot description and convert joint_states to tf messages
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        node_namespace=namespace_,
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher_node',
        remappings=[
                ('/joint_states', '/{}/joint_states'.format(namespace_))
        ],
        arguments=[urdf_model],
        emulate_tty=True
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(
                pkg_gazebo_ros, 'worlds', 'empty.world'), ''],
            description='SDF world file'
        ),
        # gazebo,
        spawn_rover,
        robot_state_publisher_node,
        Node(
            package='joy',
            node_namespace=namespace_,
            node_executable='joy_node',
            node_name='joy_node',
            remappings=[
                    ('joy', 'gamepad')
            ],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='gamepad_parser',
            node_namespace=namespace_,
            node_executable='gamepad_parser_node',
            node_name='gamepad_parser_node',
            output='screen',
            parameters=[gamepad_parser_config_ns],
            emulate_tty=True
        ),
        Node(
            package='locomotion_manager',
            node_namespace=namespace_,
            node_executable='locomotion_manager_node',
            node_name='locomotion_manager_node',
            output='screen',
            parameters=[locomotion_manager_config_ns],
            emulate_tty=True
        ),
        Node(
            package='locomotion_mode',
            node_namespace=namespace_,
            node_executable='stop_mode_node',
            node_name='stop_mode_node',
            output='screen',
            emulate_tty=True,
            # Parameters can be passed as dict or path to the .yaml
            parameters=[urdf_params, stop_mode_config_ns]
        ),
        Node(
            package='simple_rover_locomotion',
            node_namespace=namespace_,
            node_executable='simple_rover_locomotion_node',
            node_name='simple_rover_locomotion_node',
            output='screen',
            emulate_tty=True,
            parameters=[urdf_params, simple_rover_locomotion_config_ns]
        )

    ])
