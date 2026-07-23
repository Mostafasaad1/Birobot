# Copyright 2026 Birobot Project
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Birobot Project nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch file for displaying the Birobot dual-arm UR10e workcell."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def launch_setup(context, *args, **kwargs):
    """Set up nodes with context-evaluated launch arguments."""
    start_rviz = LaunchConfiguration('start_rviz')

    mappings = {
        'arm1_x': LaunchConfiguration('arm1_x').perform(context),
        'arm1_y': LaunchConfiguration('arm1_y').perform(context),
        'arm1_z': LaunchConfiguration('arm1_z').perform(context),
        'arm1_roll': LaunchConfiguration('arm1_roll').perform(context),
        'arm1_pitch': LaunchConfiguration('arm1_pitch').perform(context),
        'arm1_yaw': LaunchConfiguration('arm1_yaw').perform(context),
        'arm2_x': LaunchConfiguration('arm2_x').perform(context),
        'arm2_y': LaunchConfiguration('arm2_y').perform(context),
        'arm2_z': LaunchConfiguration('arm2_z').perform(context),
        'arm2_roll': LaunchConfiguration('arm2_roll').perform(context),
        'arm2_pitch': LaunchConfiguration('arm2_pitch').perform(context),
        'arm2_yaw': LaunchConfiguration('arm2_yaw').perform(context),
    }

    pkg_share = FindPackageShare('birobot_description').find(
        'birobot_description'
    )
    xacro_file = os.path.join(pkg_share, 'urdf', 'birobot.urdf.xacro')

    doc = xacro.process_file(xacro_file, mappings=mappings)
    robot_description = {'robot_description': doc.toxml()}

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    initial_positions_file = os.path.join(
        pkg_share, 'config', 'initial_positions.yaml'
    )
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[initial_positions_file],
    )

    rviz_config_file = os.path.join(
        pkg_share, 'config', 'rviz', 'birobot.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    return [rsp_node, jsp_gui_node, rviz_node]


def generate_launch_description():
    """Generate launch description for birobot_description."""
    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz', default_value='true', description='Start RViz2'
        ),
        DeclareLaunchArgument(
            'arm1_x',
            default_value='-0.600',
            description='Arm 1 X mount offset',
        ),
        DeclareLaunchArgument(
            'arm1_y',
            default_value='0.000',
            description='Arm 1 Y mount offset',
        ),
        DeclareLaunchArgument(
            'arm1_z',
            default_value='0.000',
            description='Arm 1 Z mount offset',
        ),
        DeclareLaunchArgument(
            'arm1_roll',
            default_value='0.000',
            description='Arm 1 roll offset',
        ),
        DeclareLaunchArgument(
            'arm1_pitch',
            default_value='0.000',
            description='Arm 1 pitch offset',
        ),
        DeclareLaunchArgument(
            'arm1_yaw',
            default_value='0.000',
            description='Arm 1 yaw offset',
        ),
        DeclareLaunchArgument(
            'arm2_x',
            default_value='0.600',
            description='Arm 2 X mount offset',
        ),
        DeclareLaunchArgument(
            'arm2_y',
            default_value='0.000',
            description='Arm 2 Y mount offset',
        ),
        DeclareLaunchArgument(
            'arm2_z',
            default_value='0.000',
            description='Arm 2 Z mount offset',
        ),
        DeclareLaunchArgument(
            'arm2_roll',
            default_value='0.000',
            description='Arm 2 roll offset',
        ),
        DeclareLaunchArgument(
            'arm2_pitch',
            default_value='0.000',
            description='Arm 2 pitch offset',
        ),
        DeclareLaunchArgument(
            'arm2_yaw',
            default_value='3.141592653589793',
            description='Arm 2 yaw offset',
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
