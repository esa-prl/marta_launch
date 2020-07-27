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
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    robot_description = LaunchConfiguration('robot_description')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_simulator = LaunchConfiguration('use_simulator')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    world = LaunchConfiguration('world')

    # ## ROBOT MODEL
    # Load XACRO and parse to URDF
    xacro_model_name = 'marta.xacro'
    xacro_model_path = os.path.join(rover_config_dir, 'urdf', xacro_model_name)

    # Parse XACRO file to URDF
    # TODO: Use LaunchConfigurations arguments to set parameters in URDF. How can the parameters be read out here???
    urdf_model_path = to_urdf(xacro_model_path, mappings={'use_ptu': 'true'})

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

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(rover_config_dir, 'rviz', 'gamepad_sim.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_gazebo_gui_cmd = DeclareLaunchArgument(
        'use_gazebo_gui',
        default_value='True',
        description='Whether to execute gzclient)')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # TODO(orduno) Switch back once ROS argument passing has been fixed upstream
        #              https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/91
        # EMPTY WORLD
        # default_value=os.path.join(rover_config_dir, 'worlds', 'empty.world'),
        # MARS YARD
        default_value=os.path.join(rover_config_dir, 'worlds', 'mars_yard.world'),
        description='Full path to world model file to load')

    return LaunchDescription([
        # This makes the outpus appearing but WARN and ERROR are not printed YLW and RED
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        # Parameter Declarations
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_config_file_cmd,
        declare_robot_description_cmd,
        declare_rviz_config_file_cmd,
        declare_use_rviz_cmd,
        declare_use_simulator_cmd,
        declare_use_gazebo_gui_cmd,
        declare_world_cmd,

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(marta_launch_dir, 'locomotion.launch.py')),
                                 launch_arguments={
                                     'namespace': namespace,
                                     'use_sim_time': use_sim_time,
                                     'config_file': config_file,
                                     'robot_description': robot_description
                                 }.items()),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(marta_launch_dir, 'simulation.launch.py')),
                                 launch_arguments={
                                     'namespace': namespace,
                                     'use_sim_time': use_sim_time,
                                     'use_simulator': use_simulator,
                                     'use_gazebo_gui': use_gazebo_gui,
                                     'robot_description': robot_description
                                 }.items()),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static')])
    ])
