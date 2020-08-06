import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros
import lifecycle_msgs

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pd_config_dir = os.path.join(get_package_share_directory('platform_driver_ethercat_ros2'), 'config')

    # Launch configurations
    pd_config_file = LaunchConfiguration('pd_config_file')

    # Launch declarations
    declare_pd_config_file_cmd = DeclareLaunchArgument(
        'pd_config_file',
        default_value=os.path.join(pd_config_dir, 'marta.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')


    # Create pd node
    pd_node = launch_ros.actions.LifecycleNode(
            package = 'platform_driver_ethercat_ros2',
            executable = 'platform_driver_ethercat_node',
            name = 'platform_driver_ethercat_node',
            output = 'screen',
            # arguments = ['--ros-args', '--log-level', 'debug'],
            parameters = [{'config_file': pd_config_file}]
    )

    # Make the pd node take the 'configure' transition
    pd_configure_event = launch.actions.EmitEvent(
        event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(pd_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
        )
    )

    # Make the pd node take the 'activate' transition
    pd_activate_event = launch.actions.EmitEvent(
        event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(pd_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
        )
    )

    # When the pd node reaches the 'inactive' state from the 'unconfigured' state, make it take the 'activate' transition
    pd_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node = pd_node,
            start_state = 'configuring',
            goal_state = 'inactive',
            entities = [pd_activate_event]
        )
    )

    ld = launch.LaunchDescription()

    ld.add_action(declare_pd_config_file_cmd)
    ld.add_action(pd_inactive_state_handler)
    ld.add_action(pd_node)
    ld.add_action(pd_configure_event)

    return ld
