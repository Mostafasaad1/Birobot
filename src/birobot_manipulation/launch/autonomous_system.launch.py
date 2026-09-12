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

"""Master launch for full autonomous dual-arm collaborative workflow."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def generate_launch_description():
    """Generate launch description for the master autonomous system."""
    pkg_moveit = get_package_share_directory('birobot_moveit_config')
    pkg_perception = get_package_share_directory('birobot_perception')
    pkg_manipulation = get_package_share_directory('birobot_manipulation')

    launch_bt = LaunchConfiguration('launch_bt')
    launch_perception = LaunchConfiguration('launch_perception')

    declared_arguments = [
        DeclareLaunchArgument(
            'launch_bt',
            default_value='true',
            description='Launch BehaviorTree.CPP v4 collaborative coordinator',
        ),
        DeclareLaunchArgument(
            'launch_perception',
            default_value='true',
            description='Launch 3D perception pipeline (RANSAC + PCA)',
        ),
        DeclareLaunchArgument(
            'randomize',
            default_value='true',
            description='Randomize red object spawn pose in Gazebo',
        ),
    ]

    # 1. Base Gazebo Sim + MoveIt 2 + RViz2 + ros2_control + Drop-off Bin
    gazebo_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, 'launch', 'gazebo_random.launch.py')
        ),
        launch_arguments={'randomize': LaunchConfiguration('randomize')}.items(),
    )

    # 2. 3D Perception Managed Lifecycle Node
    params_file = os.path.join(
        pkg_perception, 'config', 'perception_params.yaml'
    )
    perception_node = Node(
        package='birobot_perception',
        executable='birobot_perception_node',
        name='birobot_perception_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
        condition=IfCondition(launch_perception),
    )

    # Load robot model descriptions and kinematics for MoveGroupInterface
    birobot_desc_share = get_package_share_directory('birobot_description')
    xacro_file = os.path.join(
        birobot_desc_share, 'urdf', 'birobot.urdf.xacro'
    )
    doc = xacro.process_file(
        xacro_file,
        mappings={
            'sim_ignition': 'true',
            'use_fake_hardware': 'false',
        }
    )
    robot_description_str = doc.toxml()
    robot_description = {'robot_description': robot_description_str}

    srdf_file = os.path.join(pkg_moveit, 'config', 'birobot.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_file = os.path.join(pkg_moveit, 'config', 'kinematics.yaml')
    with open(kinematics_file, 'r') as f:
        robot_description_kinematics = {
            'robot_description_kinematics': yaml.safe_load(f)
        }

    joint_limits_file = os.path.join(pkg_moveit, 'config', 'joint_limits.yaml')
    with open(joint_limits_file, 'r') as f:
        robot_description_planning = {
            'robot_description_planning': yaml.safe_load(f)
        }

    # 3. BehaviorTree.CPP v4 Mission Coordinator
    bt_xml_file = os.path.join(
        pkg_manipulation, 'config', 'bt_trees', 'collaborative_handover.xml'
    )
    bt_coordinator_node = Node(
        package='birobot_manipulation',
        executable='birobot_bt_coordinator_node',
        name='birobot_bt_coordinator',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {'bt_xml_file': bt_xml_file},
            {'use_sim_time': True},
        ],
        condition=IfCondition(launch_bt),
    )

    # Sequence perception and BT coordinator after simulation stabilization
    sequenced_nodes = TimerAction(
        period=22.0,
        actions=[
            perception_node,
            bt_coordinator_node,
        ],
    )

    return LaunchDescription(
        declared_arguments + [
            gazebo_moveit,
            sequenced_nodes,
        ]
    )
