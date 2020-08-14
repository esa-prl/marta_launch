import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_helpers import to_urdf

from launch_ros.actions import Node


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
    use_simulator = LaunchConfiguration('use_simulator')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    world_path = LaunchConfiguration('world_path')

    # Load XACRO and parse to URDF
    xacro_model_name = 'marta.xacro'
    xacro_model_path = os.path.join(rover_config_dir, 'urdf', xacro_model_name)

    # Parse XACRO file to URDF
    # TODO: Use LaunchConfigurations arguments to set parameters in URDF. How can the parameters be read out here???
    urdf_model_path, robot_desc = to_urdf(xacro_model_path, mappings={'use_ptu': 'true'})

    # Launch declarations
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true.')

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

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(rover_config_dir, 'rviz', 'gamepad_sim.rviz'),
        description='Full path to the RVIZ config file to use.')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ.')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator.')

    declare_use_gazebo_gui_cmd = DeclareLaunchArgument(
        'use_gazebo_gui',
        default_value='True',
        description='Whether to execute gazebo gui (gzclient).')

    declare_world_path_cmd = DeclareLaunchArgument(
        'world_path',
        default_value=os.path.join(rover_config_dir, 'worlds', 'mars_yard.world'),
        description='Full path to world model file to load.')

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        # Set env var to print messages colored. The ANSI color codes will appear in a log.
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        # Parameter Declarations
        declare_use_sim_time_cmd,
        declare_config_file_cmd,
        declare_robot_description_cmd,
        declare_rviz_config_file_cmd,
        declare_urdf_path_cmd,
        declare_use_rviz_cmd,
        declare_use_simulator_cmd,
        declare_use_gazebo_gui_cmd,
        declare_world_path_cmd,

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(marta_launch_dir, 'locomotion.launch.py'))),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(marta_launch_dir, 'simulation.launch.py'))),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static')])
    ])
