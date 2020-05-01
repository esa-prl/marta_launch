from launch import LaunchDescription
from launch_ros.actions import Node
from launch_helpers import get_ws_src_directory, add_namespace_to_yaml, to_urdf
import os
from ament_index_python.packages import get_package_share_directory
import launch_testing
import pytest
import unittest
import rclpy
import sensor_msgs.msg
import time

namespace_ = 'marta'

# Get package src path based on a package name. Make sure the package is installed from source.
ros2_ws_src = get_ws_src_directory('locomotion_manager')

# This is necessary to get unbuffered output from the process under test
proc_env = os.environ.copy()
proc_env['PYTHONUNBUFFERED'] = '1'


@pytest.mark.launch_test
def generate_test_description():

    # ROBOT MODEL
    # Load XACRO and parse to URDF
    pkg_rover_config = get_package_share_directory('rover_config')
    xacro_model_name = "marta.xacro"
    xacro_model_path = os.path.join(pkg_rover_config, 'urdf', xacro_model_name)
    # Parse XACRO file to URDF
    urdf_model_path = to_urdf(xacro_model_path)
    urdf_params = {'urdf_model_path': urdf_model_path}

    # PARAMETERS
    # Individual Parameter files
    locomotion_manager_config = os.path.join(
        ros2_ws_src, 'locomotion_manager', 'config', 'locomotion_manager.yaml')
    simple_rover_locomotion_config = os.path.join(
        ros2_ws_src, 'simple_rover_locomotion', 'config', 'robot_poses.yaml')
    stop_mode_config = os.path.join(ros2_ws_src, 'locomotion_mode', 'config', 'stop_mode.yaml')
    gamepad_parser_config = os.path.join(
        ros2_ws_src, 'gamepad_parser', 'config', 'gamepad_parser.yaml')

    # Add namespace to the yaml file
    locomotion_manager_config_ns = add_namespace_to_yaml(namespace_, locomotion_manager_config)
    simple_rover_locomotion_config_ns = add_namespace_to_yaml(
        namespace_, simple_rover_locomotion_config)
    stop_mode_config_ns = add_namespace_to_yaml(namespace_, stop_mode_config)
    gamepad_parser_config_ns = add_namespace_to_yaml(namespace_, gamepad_parser_config)

    gamepad_parser_node = Node(
        package='gamepad_parser',
        node_namespace=namespace_,
        node_executable='gamepad_parser_node',
        node_name='gamepad_parser_node',
        output='screen',
        parameters=[gamepad_parser_config_ns],
        emulate_tty=True
    )

    locomotion_manager_node = Node(
        package='locomotion_manager',
        node_namespace=namespace_,
        node_executable='locomotion_manager_node',
        node_name='locomotion_manager_node',
        output='screen',
        parameters=[locomotion_manager_config_ns],
        emulate_tty=True
    )

    stop_mode_node = Node(
        package='locomotion_mode',
        node_namespace=namespace_,
        node_executable='stop_mode_node',
        node_name='stop_mode_node',
        output='screen',
        parameters=[urdf_params, stop_mode_config_ns],
        emulate_tty=True
    )

    simple_rover_locomotion_node = Node(
        package='simple_rover_locomotion',
        node_namespace=namespace_,
        node_executable='simple_rover_locomotion_node',
        node_name='simple_rover_locomotion_node',
        output='screen',
        emulate_tty=True,
        # Parameters can be passed as dict or path to the .yaml
        parameters=[urdf_params, simple_rover_locomotion_config_ns]
    )

    return(
        LaunchDescription([
            gamepad_parser_node,
            locomotion_manager_node,
            stop_mode_node,
            simple_rover_locomotion_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'gamepad_parser': gamepad_parser_node,
            'locomotion_manager': locomotion_manager_node,
            'stop_mode': stop_mode_node,
            'simple_rover_locomotion': simple_rover_locomotion_node,
        }
    )


class TestSystem(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create ROS node for input
        self.node = rclpy.create_node('test_input_node', namespace=namespace_)

    def tearDown(self):
        self.node.destroy_node()

    def test_mode_change(self, proc_output, proc_info):
        pub = self.node.create_publisher(
            sensor_msgs.msg.Joy,
            'gamepad',
            10
        )

        try:
            # Wait a bit to start all required nodes
            end_time = time.time() + 5
            while time.time() < end_time:
                pass

            # Send messages that emulates the button pressed
            # to activate simple_rover_locomotion
            rclpy.spin_once(self.node, timeout_sec=1.0)
            msg = sensor_msgs.msg.Joy()
            msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            pub.publish(msg)

            rclpy.spin_once(self.node, timeout_sec=1.0)

            msg.buttons = [1, 0, 0, 0, 0, 0, 0, 0, 0]
            msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            pub.publish(msg)

            rclpy.spin_once(self.node, timeout_sec=1.0)

            msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            pub.publish(msg)

            rclpy.spin_once(self.node, timeout_sec=1.0)

            # Wait for the notification that the required mode was set
            proc_output.assertWaitFor(
                'Set simple_rover_locomotion_node as first active mode', timeout=10)

        finally:
            self.node.destroy_publisher(pub)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self):
        launch_testing.asserts.assertExitCodes(self.proc_info)
