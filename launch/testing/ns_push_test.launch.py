"""Launch file for testing of namespace pushing."""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)

from nav2_common.launch import RewrittenYaml

from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    marta_launch_dir = os.path.join(get_package_share_directory('marta_launch'), 'launch')

    stereo_ns = LaunchConfiguration('stereo_ns')
    declare_stereo_ns_cmd = DeclareLaunchArgument(
            name='stereo_ns', default_value='/loc_cam',
            description='Namespace of camera in which topics ~/left/image_raw and ~/right/image_raw are published.'
        )
    namespace = LaunchConfiguration('namespace')

    declare_ns_cmd = DeclareLaunchArgument(
            name='namespace', default_value='/test',
            description='Test Namespace'
        )

    ld = LaunchDescription([

        declare_stereo_ns_cmd,
        declare_ns_cmd,

        # Pushing of Namespace does not push back the composable nodes launched in stereo_image_proc
        GroupAction([
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(marta_launch_dir, 'stereo_image_proc.launch.py')),
                launch_arguments={
                        # 'namespace': stereo_ns,
                        # 'use_sim_time': False,
                        # 'use_system_default_qos': True,
                        # 'approximate_sync': True
                        }.items()
                )
                ])
            ])

    return ld
