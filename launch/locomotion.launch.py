"""Launch file for locomotion pipeline of MaRTA."""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument  # , SetEnvironmentVariable
# from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_helpers import to_urdf

from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml

# TODO: DO NOT USE THIS. Should be replaced w/ launch config.
namespace_ = ''


def generate_launch_description():
    """Launch description."""
    # Get the directories
    rover_config_dir = get_package_share_directory('rover_config')

    # ## ROBOT MODEL
    # Load XACRO and parse to URDF
    xacro_model_name = 'marta.xacro'
    xacro_model_path = os.path.join(rover_config_dir, 'urdf', xacro_model_name)

    # Parse XACRO file to URDF
    urdf_model_path = to_urdf(xacro_model_path)

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    robot_description = LaunchConfiguration('robot_description')

    # Launch declarations
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(rover_config_dir, 'config', 'marta.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description',
        default_value=urdf_model_path,
        description='Full path to robot urdf file.')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'robot_description': robot_description}

    configured_params = RewrittenYaml(
        source_file=config_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        # Launch Arguments
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_config_file_cmd,
        declare_robot_description_cmd,

        # Nodes
        Node(
            package='robot_state_publisher',
            node_namespace=namespace_,
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher_node',
            remappings=[
                    ('/joint_states', os.path.join(namespace_, '/joint_states'))
            ],
            arguments=[robot_description],
            # TODO: Remove arguments once the robot_desc. works via parameters
            #       Should be with Foxy
            parameters=[configured_params],
            emulate_tty=True
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
            parameters=[configured_params],
            emulate_tty=True
        ),
        Node(
            package='gamepad_parser',
            node_namespace=namespace_,
            node_executable='gamepad_parser_node',
            node_name='gamepad_parser_node',
            output='screen',
            remappings=[(os.path.join(namespace_, '/rover_motion_cmd'), '/cmd_vel')],
            parameters=[configured_params],
            emulate_tty=True
        ),
        Node(
            package='locomotion_manager',
            node_namespace=namespace_,
            node_executable='locomotion_manager_node',
            node_name='locomotion_manager_node',
            output='screen',
            parameters=[configured_params],
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
            parameters=[configured_params]
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
            parameters=[configured_params]
        )
    ])
