"""Launch the rover simulation including tf node."""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_helpers import to_urdf

from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    # Load Directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    rover_config_dir = get_package_share_directory('rover_config')

    worlds_dir = os.path.join(rover_config_dir, 'worlds/')

    # ## Parameters
    # Create urdf file from xacro and gazebo file from the package rover_config
    xacro_model = os.path.join(rover_config_dir, 'urdf', 'marta.xacro')
    urdf_model_path, robot_desc = to_urdf(xacro_model)

    # Create the launch configuration variables
    config_file = LaunchConfiguration('config_file')
    robot_description = LaunchConfiguration('robot_description')
    urdf_path = LaunchConfiguration('urdf_path')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world_name = LaunchConfiguration('world_name')
    world_path = LaunchConfiguration('world_path')

    # Create the launch declarations
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(rover_config_dir, 'config', 'marta.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_desc,
        description='Full path to robot urdf file.')

    declare_urdf_path_cmd = DeclareLaunchArgument(
        'urdf_path',
        default_value=urdf_model_path,
        description='Full path to robot urdf file.')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true.')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator.')

    declare_use_gazebo_gui_cmd = DeclareLaunchArgument(
        'use_gazebo_gui',
        default_value='True',
        description='Whether to execute the gui (gzclient).')

    declare_world_name_cmd = DeclareLaunchArgument(
        'world_name',
        default_value='tb3_world.model',
        description='SDF world file name.')

    declare_world_path_cmd = DeclareLaunchArgument(
        'world_path',
        default_value=[worlds_dir, world_name],
        description='SDF world file path including name.')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'robot_description': robot_description}

    configured_params = RewrittenYaml(
        source_file=config_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # Gazebo launch
    # Starts the gzserver (handles computations) and gzclient (handles visualization)
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': use_gazebo_gui,
                          'server': use_simulator,
                          'world': world_path}.items()
    )

    # Spawn rover
    spawn_rover_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        namespace='',
        arguments=['-entity',
                   'marta',
                   '-x', '1.5', '-y', '1', '-z', '2',
                   '-file', urdf_model_path,
                   '-reference_frame', 'world']
    )

    # Node to convert odometry message to tf message
    odom_to_tf_cmd = Node(
        package='odom_to_tf',
        executable='odom_to_tf_node',
        name='odom_to_tf_node',
        parameters=[configured_params],
    )

    # Static tf from odom to base_link
    odom_to_base_link_cmd = Node(package='tf2_ros',
                                 executable='static_transform_publisher',
                                 name='base_link_broadcaster',
                                 arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
                                 parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription([
        # Set env var to print messages colored. The ANSI color codes will appear in a log.
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        # Launch Arguments
        declare_config_file_cmd,
        declare_robot_description_cmd,
        declare_urdf_path_cmd,
        declare_use_gazebo_gui_cmd,
        declare_use_sim_time_cmd,
        declare_use_simulator_cmd,
        declare_world_name_cmd,
        declare_world_path_cmd,
        # Start Nodes
        gazebo_cmd,
        spawn_rover_cmd,
        odom_to_tf_cmd,
        odom_to_base_link_cmd
    ])
