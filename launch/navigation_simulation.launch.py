# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    marta_launch_dir = os.path.join(get_package_share_directory('marta_launch'), 'launch')
    rover_config_dir = get_package_share_directory('rover_config')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_simulator = LaunchConfiguration('use_simulator')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    world_path = LaunchConfiguration('world_path')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(rover_config_dir, 'maps', 'tb3_world_big.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(rover_config_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launchde nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_distance.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(rover_config_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_gazebo_gui_cmd = DeclareLaunchArgument(
        'use_gazebo_gui',
        default_value='True',
        description='Whether to execute gzclient)')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_world_path_cmd = DeclareLaunchArgument(
        'world_path',
        # TODO(orduno) Switch back once ROS argument passing has been fixed upstream
        #              https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/91

        # TURTLEBOT EXAMPLE
        # default_value=os.path.join(rover_config_dir, 'worlds', 'tb3_world.model'),
        # EMPTY WORLD
        default_value=os.path.join(rover_config_dir, 'worlds', 'empty.world'),
        description='Full path to world model file to load')

    # Specify the actions
    locomotion_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(marta_launch_dir, 'locomotion.launch.py')))

    simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(marta_launch_dir, 'simulation.launch.py')))

    # Start Nav2 Stack
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(marta_launch_dir, 'nav2_bringup_launch.py')),
        # PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2_bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart}.items())

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # Use this option to see all output from rviz:
        # output='screen',
        # Use this option to supress messages of missing tfs,
        # at startup of rviz and gazebo:
        output={'stdout':'log'},
        arguments=['-d', rviz_config_file],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose')])

    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    # Create the launch description and populate
    return LaunchDescription([
        # Set env var to print messages colored. The ANSI color codes will appear in a log.
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        # Declare the launch options
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_bt_xml_cmd,
        declare_autostart_cmd,

        declare_rviz_config_file_cmd,
        declare_use_simulator_cmd,
        declare_use_rviz_cmd,
        declare_use_gazebo_gui_cmd,
        declare_world_path_cmd,

        # Add any conditioned actions
        start_rviz_cmd,

        # Add other nodes and processes we need
        exit_event_handler,

        # Add the actions to launch all of the navigation nodes
        locomotion_cmd,
        simulation_cmd,
        bringup_cmd
    ])
