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

"""Launch file for autonomous vision-guided pick-and-place coordinator."""

import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_moveit = get_package_share_directory('birobot_moveit_config')
    pkg_manipulation = get_package_share_directory('birobot_manipulation')
    pkg_description = get_package_share_directory('birobot_description')

    default_bt_xml = os.path.join(
        pkg_manipulation, 'config', 'bt_trees', 'auto_pick_place.xml'
    )

    bt_xml_file = LaunchConfiguration('bt_xml_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declared_arguments = [
        DeclareLaunchArgument(
            'bt_xml_file',
            default_value=default_bt_xml,
            description='Path to Behavior Tree XML file for autonomous pick and place',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true',
        ),
    ]

    # Load robot description (URDF/Xacro)
    xacro_file = os.path.join(pkg_description, 'urdf', 'birobot.urdf.xacro')
    doc = xacro.process_file(
        xacro_file,
        mappings={
            'sim_ignition': 'true',
            'use_fake_hardware': 'false',
        },
    )
    robot_description = {'robot_description': doc.toxml()}

    # Load semantic description (SRDF)
    srdf_file = os.path.join(pkg_moveit, 'config', 'birobot.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # Load kinematics configuration
    kinematics_file = os.path.join(pkg_moveit, 'config', 'kinematics.yaml')
    with open(kinematics_file, 'r') as f:
        robot_description_kinematics = {
            'robot_description_kinematics': yaml.safe_load(f)
        }

    # Load joint limits
    joint_limits_file = os.path.join(pkg_moveit, 'config', 'joint_limits.yaml')
    with open(joint_limits_file, 'r') as f:
        robot_description_planning = {
            'robot_description_planning': yaml.safe_load(f)
        }

    coordinator_node = Node(
        package='birobot_manipulation',
        executable='auto_pick_coordinator_node',
        name='auto_pick_coordinator',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {'bt_xml_file': bt_xml_file},
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription(declared_arguments + [coordinator_node])
