import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,
                            IncludeLaunchDescription, SetEnvironmentVariable)
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
    use_rviz = LaunchConfiguration('use_rviz')
    use_simulator = LaunchConfiguration('use_simulator')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    world = LaunchConfiguration('world')
    use_loc_cam = LaunchConfiguration('use_loc_cam')
    stereo_cam_name = LaunchConfiguration('stereo_cam_name')
    use_stereo_view = LaunchConfiguration('use_stereo_view')

    # ## ROBOT MODEL
    # Load XACRO and parse to URDF
    xacro_model_name = 'marta.xacro'
    xacro_model_path = os.path.join(rover_config_dir, 'urdf', xacro_model_name)

    # Parse XACRO file to URDF
    urdf_model_path = to_urdf(xacro_model_path)

    # Launch declarations
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

    declare_use_loc_cam_cmd = DeclareLaunchArgument(
        'use_loc_cam',
        default_value='True',
        description='Whether to process loc_cam data.)')

    declare_stereo_cam_name_cmd = DeclareLaunchArgument(
        'stereo_cam_name',
        default_value='/loc_cam',
        description='Name of the stereo cam to be used.')

    declare_use_stereo_view_cmd = DeclareLaunchArgument(
        'use_stereo_view',
        default_value='True',
        description='Option to display stereo images.')

    start_locomotion_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(marta_launch_dir, 'locomotion.launch.py')))

    start_simulation_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(marta_launch_dir, 'simulation.launch.py')))

    start_stereo_image_proc_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(marta_launch_dir, 'stereo_image_proc.launch.py')),)

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
        declare_use_simulator_cmd,
        declare_use_gazebo_gui_cmd,
        declare_world_cmd,
        declare_use_loc_cam_cmd,
        declare_stereo_cam_name_cmd,
        declare_use_stereo_view_cmd,

        # Start Nodes and Launch files
        start_locomotion_cmd,
        start_simulation_cmd,
        start_stereo_image_proc_cmd,
        rviz_cmd,
    ])
