from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

import xacro
import tempfile

namespace_ = 'marta'


def generate_launch_description():

    ## Individual Parameter files
    ## TODO: Config files are installed. Thus they can not be changed without recompiling. Might be better to find the repository instead.
    gamepad_parser_dir = get_package_share_directory('gamepad_parser')
    gamepad_parser_config = os.path.join(gamepad_parser_dir, 'gamepad_parser.yaml')

    ## Add namespace to the yaml file
    gamepad_parser_config_ns = add_namespace_to_yaml(namespace_, gamepad_parser_config)

    with open(gamepad_parser_config_ns, 'r') as f:
        print(f.read())


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
        # Use this or the joint_state_publisher to publish dummy values for joints.
        # Node(
        # 	package='tf2_ros',
        # 	node_executable='static_transform_publisher',
        # 	node_name='link_back_left_broadcaster',
        # 	arguments=['0 0 0 0 0 0 0 base_link link_back_left']
        # ),
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


def to_urdf(xacro_path, urdf_path=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * urdf_path -- the path to the urdf file
    """
    # If no URDF path is given, use a temporary file
    if urdf_path is None:
        urdf_path = tempfile.mktemp(prefix="%s_" %
                                    os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path)
    # open the output file
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path  # Return path to the urdf file




def add_namespace_to_yaml(namespace, yaml_path, ns_yaml_path=None):
    """Make config files reusable in multiple namespaces by generating a new yaml with a namespace appended
    * yaml_path -- the path to the yaml file
    * ns_yaml_path -- the path to the namespaced yaml file
    """
    # If no YAML path is given, use a temporary file
    if ns_yaml_path is None:
        ns_yaml_path = tempfile.mktemp(prefix="%s_" %
                                    os.path.basename(yaml_path))

    # open and process file
    with open(yaml_path, 'r') as yaml_file, open(ns_yaml_path, 'w') as ns_yaml_file:
        ns_yaml_file.write('# Temporary Namespace\n')     
        ns_yaml_file.write(namespace + ':\n')     
        for line in yaml_file:
            ns_yaml_file.write("  " + line)
    return ns_yaml_path  # Return path to the urdf file

