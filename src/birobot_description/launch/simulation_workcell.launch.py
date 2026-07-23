# Copyright 2026 Birobot Project
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

"""Launch Gazebo simulation workcell with 3D depth camera and table objects."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def launch_setup(context, *args, **kwargs):
    """Set up nodes with context-evaluated launch arguments."""
    spawn_objects = LaunchConfiguration('spawn_objects')

    pkg_share = FindPackageShare('birobot_description').find('birobot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'birobot.urdf.xacro')

    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Gazebo server and client
    gazebo_share = FindPackageShare('gazebo_ros').find('gazebo_ros')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
        )
    )

    # Spawn birobot model into Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_birobot',
        arguments=['-entity', 'birobot', '-topic', 'robot_description', '-z', '0.0'],
        output='screen',
    )

    # Spawn test object 1 (Irregular object 1) on table
    spawn_object_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_test_object_1',
        arguments=[
            '-entity', 'irregular_object_1',
            '-xml', """<sdf version="1.6">
              <model name="irregular_object_1">
                <pose>0.10 0.05 0.85 0 0 0.4</pose>
                <link name="link">
                  <inertial>
                    <mass>0.5</mass>
                    <inertia><ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.002</iyy><iyz>0</iyz><izz>0.002</izz></inertia>
                  </inertial>
                  <visual name="visual">
                    <geometry><box><size>0.15 0.08 0.06</size></box></geometry>
                    <material><script><name>Gazebo/Red</name></script></material>
                  </visual>
                  <collision name="collision">
                    <geometry><box><size>0.15 0.08 0.06</size></box></geometry>
                  </collision>
                </link>
              </model>
            </sdf>""",
        ],
        output='screen',
        condition=IfCondition(spawn_objects),
    )

    # Spawn test object 2 (Irregular object 2) on table
    spawn_object_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_test_object_2',
        arguments=[
            '-entity', 'irregular_object_2',
            '-xml', """<sdf version="1.6">
              <model name="irregular_object_2">
                <pose>-0.15 -0.10 0.85 0 0 -0.8</pose>
                <link name="link">
                  <inertial>
                    <mass>0.4</mass>
                    <inertia><ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz></inertia>
                  </inertial>
                  <visual name="visual">
                    <geometry><box><size>0.18 0.06 0.05</size></box></geometry>
                    <material><script><name>Gazebo/Blue</name></script></material>
                  </visual>
                  <collision name="collision">
                    <geometry><box><size>0.18 0.06 0.05</size></box></geometry>
                  </collision>
                </link>
              </model>
            </sdf>""",
        ],
        output='screen',
        condition=IfCondition(spawn_objects),
    )

    return [rsp_node, gazebo, spawn_robot, spawn_object_1, spawn_object_2]


def generate_launch_description():
    """Generate launch description for Gazebo simulation workcell."""
    declared_arguments = [
        DeclareLaunchArgument(
            'spawn_objects',
            default_value='true',
            description='Spawn irregular test objects on table surface',
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
