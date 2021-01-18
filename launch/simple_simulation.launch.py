import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_helpers import to_urdf

from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    marta_launch_dir = os.path.join(get_package_share_directory('marta_launch'), 'launch')
    rover_config_dir = os.path.join(get_package_share_directory('rover_config'))

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    robot_description = LaunchConfiguration('robot_description')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_path = LaunchConfiguration('urdf_path')
    use_rviz = LaunchConfiguration('use_rviz')

    # ## ROBOT MODEL
    # Load XACRO and parse to URDF
    xacro_model_name = 'marta.xacro'
    xacro_model_path = os.path.join(rover_config_dir, 'urdf', xacro_model_name)

    # Parse XACRO file to URDF
    urdf_model_path, robot_desc = to_urdf(xacro_model_path)

    # Launch declarations
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(rover_config_dir, 'config', 'marta.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_urdf_path_cmd = DeclareLaunchArgument(
        'urdf_path',
        default_value=urdf_model_path,
        description='Full path to robot urdf file.')

    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_desc,
        description='Full path to robot urdf file.')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(rover_config_dir, 'rviz', 'simple_sim.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'robot_description': urdf_model_path}

    configured_params = RewrittenYaml(
        source_file=config_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # Create Node Commands
    start_locomotion_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(marta_launch_dir, 'locomotion.launch.py')))

    joint_state_pub_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_node',
        output='screen',
        parameters=[configured_params]
    )

    simple_joint_sim_cmd = Node(
        package='simple_joint_simulation',
        executable='simple_joint_simulation_node',
        name='simple_joint_simulation_node',
        output='screen',
        parameters=[configured_params]
    )

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        # Use this option to see all output from rviz:
        # output='screen',
        # Use this option to supress messages of missing tfs,
        # at startup of rviz and gazebo:
        output={'stdout':'log'},
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')])
    return LaunchDescription([
        # Set env var to print messages colored. The ANSI color codes will appear in a log.
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        # Parameter Declarations
        declare_use_sim_time_cmd,
        declare_config_file_cmd,
        declare_robot_description_cmd,
        declare_rviz_config_file_cmd,
        declare_use_rviz_cmd,
        declare_urdf_path_cmd,

        # Start Nodes
        start_locomotion_cmd,
        joint_state_pub_cmd,
        simple_joint_sim_cmd,
        rviz_cmd
    ])
