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

"""Launch file for birobot_perception managed lifecycle node."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for perception pipeline."""
    pkg_share = FindPackageShare('birobot_perception').find('birobot_perception')
    params_file = os.path.join(pkg_share, 'config', 'perception_params.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Path to perception node parameters file',
    )

    perception_node = Node(
        package='birobot_perception',
        executable='birobot_perception_node',
        name='birobot_perception_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_arg,
        perception_node,
    ])
