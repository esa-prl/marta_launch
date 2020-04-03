from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch_helpers import get_ws_src_directory, add_namespace_to_yaml, get_ws_src_directory

import os

namespace_ = 'marta'
# Get package src path based on a package name. Make sure the package is installed from source.
ros2_ws_src = get_ws_src_directory('gamepad_parser')

def generate_launch_description():

    # Individual Parameter files
    gamepad_parser_config = os.path.join(ros2_ws_src, 'gamepad_parser', 'config', 'gamepad_parser.yaml')

    # Add namespace to the yaml file
    gamepad_parser_config_ns = add_namespace_to_yaml(namespace_, gamepad_parser_config)

    # Load XACRO and parse to URDF
    pkg_rover_config = get_package_share_directory('rover_config')
    xacro_model_name = "marta.xacro"
    xacro_model_path = os.path.join(pkg_rover_config, 'urdf', xacro_model_name)

    # Parse XACRO file to URDF
    urdf_model_path = to_urdf(xacro_model_path)
    urdf_params = {'urdf_model_path': urdf_model_path}

    # Parameters for the joint_state_publisher
    joint_state_params = {'use_gui': True,
                          'rate':	 50,
                          'publish_default_velocities': True,
                          'publish_default_efforts': True,
                          'robot_description': urdf_model_path,
                          'source_list': ['/{}/joint_states_sim'.format(namespace_)]}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            node_namespace=namespace_,
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher_node',
            # output='screen',
            # remappings=[
            # 	('robot_description', 'urdf_model_path')
            # ],
            remappings=[
                    ('/joint_states', '/{}/joint_states'.format(namespace_))
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
                     '/{}/robot_description'.format(namespace_))
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
            parameters=[gamepad_parser_config_ns],
            emulate_tty=True
        ),
        Node(
            package='simple_joint_simulation',
            node_namespace=namespace_,
            node_executable='simple_joint_simulation_node',
            node_name='simple_joint_simulation_node',
            output='screen',
            emulate_tty=True
        ),

        Node(
            package='simple_rover_locomotion',
            node_namespace=namespace_,
            node_executable='simple_rover_locomotion_node',
            node_name='simple_rover_locomotion_node',
            output='screen',
            emulate_tty=True,
            parameters=[(urdf_params)]
        )
    ])

