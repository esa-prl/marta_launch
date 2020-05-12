"""Launch file for locomotion pipeline of MaRTA."""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument  # , SetEnvironmentVariable
from launch_ros.actions import Node
# from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_helpers import add_namespace_to_yaml, to_urdf

namespace_ = ''


def generate_launch_description():
    """Launch description."""
    # Get the directories
    launch_dir = get_package_share_directory('marta_launch')
    gamepad_parser_dir = get_package_share_directory('gamepad_parser')
    rover_config_dir = get_package_share_directory('rover_config')
    locomotion_manager_dir = get_package_share_directory('locomotion_manager')
    locomotion_mode_dir = get_package_share_directory('locomotion_mode')
    simple_rover_locomotion_dir = get_package_share_directory('simple_rover_locomotion')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    robot_model = LaunchConfiguration('robot_model')

    # ROBOT MODEL
    # Load XACRO and parse to URDF
    xacro_model_name = "marta.xacro"
    xacro_model_path = os.path.join(rover_config_dir, 'urdf', xacro_model_name)

    # Parse XACRO file to URDF
    urdf_model_path = to_urdf(xacro_model_path)
    urdf_params = {'urdf_model_path': urdf_model_path}

    # ## PARAMETERS
    # Individual Parameter files
    gamepad_parser_config = os.path.join(
        gamepad_parser_dir, 'gamepad_parser.yaml')
    locomotion_manager_config = os.path.join(
        locomotion_manager_dir, 'locomotion_manager.yaml')
    simple_rover_locomotion_config = os.path.join(
        simple_rover_locomotion_dir, 'config', 'robot_poses.yaml')
    stop_mode_config = os.path.join(
        locomotion_mode_dir, 'config', 'stop_mode.yaml')

    # Add namespace to the yaml file
    gamepad_parser_config_ns = add_namespace_to_yaml(namespace_, gamepad_parser_config)
    locomotion_manager_config_ns = add_namespace_to_yaml(namespace_, locomotion_manager_config)
    simple_rover_locomotion_config_ns = add_namespace_to_yaml(namespace_,
                                                              simple_rover_locomotion_config)
    stop_mode_config_ns = add_namespace_to_yaml(namespace_, stop_mode_config)

    # Parameters for the joint_state_publisher
    joint_state_params = {'use_gui': True,
                          'rate':    50,
                          'publish_default_velocities': True,
                          'publish_default_efforts': True,
                          'robot_description': urdf_model_path,
                          'source_list': [os.path.join(namespace_, 'joint_states_sim')]}

    # use_sim_time = True

    return LaunchDescription([
        # TODO: The launch arguments are actually not doing anything atm...
        # Launch Arguments
        DeclareLaunchArgument(
            'namespace', default_value=namespace_,
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'robot_model', default_value=urdf_model_path,
            description='Robot Model'),

        # Nodes
        Node(
            package='robot_state_publisher',
            node_namespace=namespace_,
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher_node',
            remappings=[
                    ('/joint_states', os.path.join(namespace_, '/joint_states'))
            ],
            arguments=[urdf_model_path],
            emulate_tty=True
        ),
        Node(
            package='joint_state_publisher',
            node_namespace=namespace_,
            node_executable='joint_state_publisher',
            remappings=[
                    ('/robot_description',
                     os.path.join(namespace_, '/robot_description'))
            ],
            node_name='joint_state_publisher_node',
            output='screen',
            parameters=[(joint_state_params)]
        ),
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
            remappings=[(os.path.join(namespace_, '/rover_motion_cmd'), '/cmd_vel')],
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
            package='simple_rover_locomotion',
            node_namespace=namespace_,
            node_executable='simple_rover_locomotion_node',
            node_name='simple_rover_locomotion_node',
            remappings=[(os.path.join(namespace_, '/rover_motion_cmd'), '/cmd_vel')],
            output='screen',
            emulate_tty=True,
            # Parameters can be passed as dict or path to the .yaml
            parameters=[urdf_params, simple_rover_locomotion_config_ns]
        ),
        Node(
            package='locomotion_mode',
            node_namespace=namespace_,
            node_executable='stop_mode_node',
            node_name='stop_mode_node',
            remappings=[(os.path.join(namespace_, '/rover_motion_cmd'), '/cmd_vel')],
            output='screen',
            emulate_tty=True,
            # Parameters can be passed as dict or path to the .yaml
            parameters=[urdf_params, stop_mode_config_ns]
        )
    ])
