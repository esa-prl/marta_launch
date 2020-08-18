"""Launch file for locomotion pipeline of MaRTA."""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_helpers import to_urdf

from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Launch description."""

    # Get the directories
    rover_config_dir = get_package_share_directory('rover_config')

    # ## ROBOT MODEL
    # Load XACRO and parse to URDF
    xacro_model_name = 'marta.xacro'
    xacro_model_path = os.path.join(rover_config_dir, 'urdf', xacro_model_name)

    # Parse XACRO file to URDF
    urdf_model_path, robot_desc = to_urdf(xacro_model_path)

    # Create the launch configuration variables
    config_file = LaunchConfiguration('config_file')
    robot_description = LaunchConfiguration('robot_description')
    urdf_path = LaunchConfiguration('urdf_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ptu = LaunchConfiguration('use_ptu')

    # Launch declarations
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(rover_config_dir, 'config', 'marta.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes.')

    declare_urdf_path_cmd = DeclareLaunchArgument(
        'urdf_path',
        default_value=urdf_model_path,
        description='Full path to robot urdf file.')

    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_desc,
        description='Full path to robot urdf file.')


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_use_ptu_cmd = DeclareLaunchArgument(
        'use_ptu',
        default_value='True',
        description='Whether to start the PTU controller')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'robot_description': robot_description,
        'urdf_path': urdf_path}

    configured_params = RewrittenYaml(
        source_file=config_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    robot_state_pub_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        remappings=[('/joint_states', '/joint_states')],
        parameters=[configured_params]
    )

    joy_cmd = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        remappings=[('joy', 'gamepad')],
        parameters=[configured_params],
    )

    gamepad_parser_cmd = Node(
        package='gamepad_parser',
        executable='gamepad_parser_node',
        name='gamepad_parser_node',
        remappings=[('/rover_motion_cmd', '/cmd_vel')],
        parameters=[configured_params],
    )

    locomotion_manager_cmd = Node(
        package='locomotion_manager',
        executable='locomotion_manager_node',
        name='locomotion_manager_node',
        parameters=[configured_params],
    )

    simple_rover_locomotion_cmd = Node(
        package='simple_rover_locomotion',
        executable='simple_rover_locomotion_node',
        name='simple_rover_locomotion_node',
        remappings=[('/rover_motion_cmd', '/cmd_vel')],
        # Parameters can be passed as dict or path to the .yaml
        parameters=[configured_params],
    )

    stop_mode_cmd = Node(
        package='locomotion_mode',
        executable='stop_mode_node',
        name='stop_mode_node',
        remappings=[('/rover_motion_cmd', '/cmd_vel')],
        # Parameters can be passed as dict or path to the .yaml
        parameters=[configured_params],
    )

    ptu_control_cmd = Node(
        condition=IfCondition(use_ptu),
        package='ptu_control',
        executable='ptu_control_node',
        name='ptu_control_node',
        parameters=[configured_params],
    )


    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        # Set env var to print messages colored. The ANSI color codes will appear in a log.
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        # Launch Arguments
        declare_config_file_cmd,
        declare_urdf_path_cmd,
        declare_robot_description_cmd,
        declare_use_sim_time_cmd,
        declare_use_ptu_cmd,

        # Start Nodes
        robot_state_pub_cmd,
        joy_cmd,
        gamepad_parser_cmd,
        locomotion_manager_cmd,
        simple_rover_locomotion_cmd,
        stop_mode_cmd,
        ptu_control_cmd
    ])
