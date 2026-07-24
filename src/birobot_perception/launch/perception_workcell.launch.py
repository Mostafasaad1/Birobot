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

"""Unified System Launch: Gazebo Sim Workcell + 3D Perception Node + RViz2 (ROS 2 Jazzy)."""

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def launch_setup(context, *args, **kwargs):
    """Set up all nodes and simulation components."""
    spawn_objects = LaunchConfiguration('spawn_objects')
    use_rviz = LaunchConfiguration('use_rviz')
    launch_perception = LaunchConfiguration('launch_perception')

    pkg_description_share = FindPackageShare('birobot_description').find('birobot_description')
    pkg_perception_share = FindPackageShare('birobot_perception').find('birobot_perception')

    # Environment variable for Gazebo Sim meshes
    vendor_dir = os.path.join(pkg_description_share, 'vendor')
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{vendor_dir}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    )

    controllers_yaml_path = '/tmp/birobot_controllers.yaml'
    original_controllers = os.path.join(pkg_description_share, 'config', 'controllers.yaml')
    if os.path.exists(original_controllers):
        import shutil
        shutil.copy(original_controllers, controllers_yaml_path)

    xacro_file = os.path.join(pkg_description_share, 'urdf', 'birobot.urdf.xacro')

    doc = xacro.process_file(
        xacro_file,
        mappings={
            'sim_ignition': 'true',
            'use_fake_hardware': 'false',
            'controllers_yaml': controllers_yaml_path if os.path.exists(controllers_yaml_path) else '',
        }
    )
    robot_description_str = doc.toxml()
    robot_description = {'robot_description': robot_description_str}

    # Save processed URDF to tempfile for ros_gz_sim create executable
    _urdf_tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.urdf', delete=False, prefix='birobot_system_'
    )
    _urdf_tmp.write(robot_description_str)
    _urdf_tmp.close()
    urdf_file = _urdf_tmp.name

    # 1. Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # 2. Gazebo Sim
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim').find('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. Clock & PointCloud Bridges
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/birobot/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/birobot/depth_camera/points/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/birobot/depth_camera/points/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen'
    )

    # 4. Spawn Birobot Model into Gazebo Sim
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_file,
            '-name', 'birobot',
            '-world', 'empty',
            '-allow_renaming', 'true',
            '-J', 'arm1_shoulder_lift_joint', '-1.5708',
            '-J', 'arm1_wrist_1_joint', '-1.5708',
            '-J', 'arm2_shoulder_lift_joint', '-1.5708',
            '-J', 'arm2_wrist_1_joint', '-1.5708',
        ],
        output='screen',
    )

    # 5. Spawn Object 1 (Red irregular box resting on table surface z = 0.08)
    obj1_sdf = """<sdf version="1.6">
      <model name="irregular_object_1">
        <pose>0 0 0 0 0 0</pose>
        <link name="link">
          <inertial>
            <mass>0.5</mass>
            <inertia><ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.002</iyy><iyz>0</iyz><izz>0.002</izz></inertia>
          </inertial>
          <visual name="visual">
            <geometry><box><size>0.15 0.08 0.06</size></box></geometry>
            <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
          </visual>
          <collision name="collision">
            <geometry><box><size>0.15 0.08 0.06</size></box></geometry>
          </collision>
        </link>
      </model>
    </sdf>"""
    _obj1_tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False, prefix='obj1_')
    _obj1_tmp.write(obj1_sdf)
    _obj1_tmp.close()

    spawn_object_1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', _obj1_tmp.name,
            '-name', 'irregular_object_1',
            '-world', 'empty',
            '-x', '0.10',
            '-y', '0.05',
            '-z', '0.08',
            '-Y', '0.4',
        ],
        output='screen',
        condition=IfCondition(spawn_objects),
    )

    # 6. Spawn Object 2 (Blue irregular box resting on table surface z = 0.075)
    obj2_sdf = """<sdf version="1.6">
      <model name="irregular_object_2">
        <pose>0 0 0 0 0 0</pose>
        <link name="link">
          <inertial>
            <mass>0.4</mass>
            <inertia><ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz></inertia>
          </inertial>
          <visual name="visual">
            <geometry><box><size>0.18 0.06 0.05</size></box></geometry>
            <material><ambient>0 0 1 1</ambient><diffuse>0 0 1 1</diffuse></material>
          </visual>
          <collision name="collision">
            <geometry><box><size>0.18 0.06 0.05</size></box></geometry>
          </collision>
        </link>
      </model>
    </sdf>"""
    _obj2_tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False, prefix='obj2_')
    _obj2_tmp.write(obj2_sdf)
    _obj2_tmp.close()

    spawn_object_2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', _obj2_tmp.name,
            '-name', 'irregular_object_2',
            '-world', 'empty',
            '-x', '-0.15',
            '-y', '-0.10',
            '-z', '0.075',
            '-Y', '-0.8',
        ],
        output='screen',
        condition=IfCondition(spawn_objects),
    )

    # 7. Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm1_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm1_joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm2_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm2_joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # 8. Perception Managed Lifecycle Node
    params_file = os.path.join(pkg_perception_share, 'config', 'perception_params.yaml')
    perception_node = Node(
        package='birobot_perception',
        executable='birobot_perception_node',
        name='birobot_perception_node',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(launch_perception),
    )

    # 9. RViz2 Visualization
    rviz_config_file = os.path.join(pkg_perception_share, 'config', 'perception.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return [
        set_gz_resource_path,
        rsp_node,
        gazebo,
        clock_bridge,
        spawn_robot,
        spawn_object_1,
        spawn_object_2,
        joint_state_broadcaster_spawner,
        arm1_controller_spawner,
        arm2_controller_spawner,
        perception_node,
        rviz_node,
    ]


def generate_launch_description():
    """Generate launch description for full Birobot system."""
    declared_arguments = [
        DeclareLaunchArgument(
            'spawn_objects',
            default_value='true',
            description='Spawn irregular test objects on table surface',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 visualization automatically',
        ),
        DeclareLaunchArgument(
            'launch_perception',
            default_value='true',
            description='Launch 3D perception lifecycle node automatically',
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
