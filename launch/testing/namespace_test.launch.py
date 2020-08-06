import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from nav2_common.launch import Node


def generate_launch_description():

    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='bla',
        description='Top-level namespace')

    print(declare_namespace_cmd.parse())

    gamepad_parser_cmd = Node(
        package='gamepad_parser',
        namespace=namespace,
        executable='gamepad_parser_node',
        name='gamepad_parser_node',
        output='screen',
        emulate_tty=True)

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(gamepad_parser_cmd)

    return ld