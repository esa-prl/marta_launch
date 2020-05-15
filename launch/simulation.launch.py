"""Launch the rover simulation including tf node."""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_helpers import to_urdf

from launch_ros.actions import Node


def generate_launch_description():

    # Load Directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    rover_config_dir = os.path.join(get_package_share_directory('rover_config'))

    # ## Parameters
    # Old namespace definition
    namespace_ = ''

    # Nav2 Tutorial World
    world_1 = os.path.join(rover_config_dir, 'worlds', 'empty_worlds', 'world_only.model')
    # Empty World
    world_2 = os.path.join(rover_config_dir, 'worlds', 'empty.world')

    # Create urdf file from xacro and gazebo file from the package rover_config
    pkg_rover_config = get_package_share_directory('rover_config')
    xacro_model = os.path.join(pkg_rover_config, 'urdf', 'marta.xacro')
    urdf_model = to_urdf(xacro_model)

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    world = LaunchConfiguration('world')

    # Create the launch declarations
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=namespace_,
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

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
        default_value=[world_2, ''],
        description='SDF world file')

    # Gazebo launch
    # Starts the gzserver (handles computations) and gzclient (handles visualization)
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': use_gazebo_gui,
                          'server': use_simulator}.items()
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
                   '-x', '-1.5', '-y', '-1', '-z', '1',
                   '-file', urdf_model,
                   '-reference_frame', 'world']
    )

    # Static tf from odom to base_link
    odom_to_base_link_cmd = Node(package='tf2_ros',
                                 node_executable='static_transform_publisher',
                                 node_name='base_link_broadcaster',
                                 arguments=['0', '0', '0.0', '0', '0', '0', 'odom', 'base_link'],
                                 parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription([
        # Launch Arguments
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_use_simulator_cmd,
        declare_use_gazebo_gui_cmd,
        declare_world_cmd,
        # Start Nodes
        gazebo_cmd,
        spawn_rover_cmd,
        odom_to_base_link_cmd
    ])
