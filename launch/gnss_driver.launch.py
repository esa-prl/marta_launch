import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_helpers import to_urdf

from launch_ros.actions import Node

def generate_launch_description():
    """Simple launch description to start gnss driver."""

    rover_config_dir = get_package_share_directory('rover_config')
    
    gnss_driver_cmd = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        parameters=[os.path.join(rover_config_dir,'config','nmea_serial_driver.yaml')]
        )

    return LaunchDescription([
		# Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        # Set env var to print messages colored. The ANSI color codes will appear in a log.
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

    	gnss_driver_cmd
    	])
