# Copyright (c) 2019, Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # TODO(jacobperron): Include image_proc launch file when it exists
    stereo_cam_name = LaunchConfiguration('stereo_cam_name')
    use_stereo_view = LaunchConfiguration('use_stereo_view')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='approximate_sync', default_value='True',
            description='Whether to use approximate synchronization of topics. Set to true if '
                        'the left and right cameras do not produce exactly synced timestamps.'
        ),
        DeclareLaunchArgument(
            name='use_system_default_qos', default_value='False',
            description='Use the RMW QoS settings for the image and camera info subscriptions.'
        ),
        DeclareLaunchArgument(
            name='stereo_cam_name', default_value='/camera',
            description='Namespace'
        ),

        DeclareLaunchArgument(
            name='use_stereo_view', default_value='True',
            description='Choice if stereo_view should be started.'
        ),

        ComposableNodeContainer(
            package='rclcpp_components', executable='component_container',
            name='stereo_image_proc_container', namespace='',
            composable_node_descriptions=[
                ComposableNode(
                    package='stereo_image_proc',
                    node_plugin='stereo_image_proc::DisparityNode',
                    name='disparity_node',
                    namespace=stereo_cam_name,
                    remappings=[
                        ('left/image_rect', 'left/image_raw'),
                        ('right/image_rect', 'right/image_raw'),
                    ],
                    parameters=[{
                        'approximate_sync': LaunchConfiguration('approximate_sync'),
                        'use_system_default_qos': LaunchConfiguration('use_system_default_qos'),
                    }]
                ),
                ComposableNode(
                    package='stereo_image_proc',
                    node_plugin='stereo_image_proc::PointCloudNode',
                    name='point_cloud_node',
                    namespace=stereo_cam_name,
                    remappings=[
                        ('left/image_rect_color', 'left/image_raw')
                    ],
                    parameters=[{
                        'approximate_sync': LaunchConfiguration('approximate_sync'),
                        'use_system_default_qos': LaunchConfiguration('use_system_default_qos'),
                    }]
                ),
            ],
        ),

        Node(
            condition=IfCondition(use_stereo_view),
            package='image_view',
            executable='stereo_view',
            name='stereo_view_node',
            arguments=['raw'],
            remappings=[
                    ('/stereo/disparity',   [stereo_cam_name, '/disparity']),
                    ('/stereo/left/image',  [stereo_cam_name, '/left/image_raw']),
                    ('/stereo/right/image', [stereo_cam_name, '/right/image_raw']),
            ],
            output='screen',
            emulate_tty=True
        ),

    ])
