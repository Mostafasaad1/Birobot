"""
gazebo.launch.py — Birobot launch bringing up Gazebo Sim + MoveIt 2 + RViz2 + ros2_control + Workspace Objects
"""

import os
import shutil
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def load_yaml(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(pkg_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    birobot_description_share = get_package_share_directory('birobot_description')
    birobot_moveit_share = get_package_share_directory('birobot_moveit_config')

    # Add vendor directory to Gazebo Sim resource path so meshes resolve
    vendor_dir = os.path.join(birobot_description_share, 'vendor')
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{vendor_dir}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    )

    # ── 1. URDF / robot_description ──────────────────────────────────────────
    original_controllers_yaml_path = os.path.join(
        birobot_description_share, 'config', 'controllers.yaml'
    )
    controllers_yaml_path = '/tmp/birobot_controllers.yaml'
    shutil.copy(original_controllers_yaml_path, controllers_yaml_path)

    xacro_file = os.path.join(birobot_description_share, 'urdf', 'birobot.urdf.xacro')
    doc = xacro.process_file(
        xacro_file,
        mappings={
            'sim_ignition': 'true',
            'use_fake_hardware': 'false',
            'controllers_yaml': controllers_yaml_path,
        }
    )
    robot_description_str = doc.toxml()
    robot_description = {'robot_description': robot_description_str}

    # Write canonical URDF for Gazebo spawn_entity
    _urdf_tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.urdf', delete=False, prefix='birobot_spawn_'
    )
    _urdf_tmp.write(robot_description_str)
    _urdf_tmp.close()
    urdf_spawn_file = _urdf_tmp.name

    # Merged YAML params for ros2_control_node
    with open(controllers_yaml_path, 'r') as f:
        controllers_config = yaml.safe_load(f)

    merged_ros2ctrl_params = {
        '/**': {'ros__parameters': {
            'robot_description': robot_description_str,
            'use_sim_time': True,
        }},
    }
    if controllers_config:
        merged_ros2ctrl_params.update(controllers_config)

    _ctrl_params_tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix='birobot_ros2ctrl_'
    )
    yaml.dump(merged_ros2ctrl_params, _ctrl_params_tmp)
    _ctrl_params_tmp.close()

    # ── MoveIt parameters ────────────────────────────────────────────────────
    srdf_file = os.path.join(birobot_moveit_share, 'config', 'birobot.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_config = f.read()
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    kinematics_yaml = load_yaml('birobot_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = (
        {'robot_description_kinematics': kinematics_yaml} if kinematics_yaml else {}
    )

    joint_limits_yaml = load_yaml('birobot_moveit_config', 'config/joint_limits.yaml')
    robot_description_planning = (
        {'robot_description_planning': joint_limits_yaml} if joint_limits_yaml else {}
    )

    ompl_planning_yaml = load_yaml('birobot_moveit_config', 'config/ompl_planning.yaml') or {}
    planning_pipelines_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
    }

    moveit_controllers_yaml = (
        load_yaml('birobot_moveit_config', 'config/moveit_controllers.yaml') or {}
    )
    trajectory_execution = {
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_manage_controllers': False,
    }

    rviz_config_file = os.path.join(birobot_moveit_share, 'config', 'moveit.rviz')

    # ── 2. Gazebo Sim & Spawners ──────────────────────────────────────────────
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_spawn_file,
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

    # Workspace Objects
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

    spawn_object_1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', obj1_sdf,
            '-name', 'irregular_object_1',
            '-world', 'empty',
            '-x', '0.10',
            '-y', '0.05',
            '-z', '0.08',
            '-Y', '0.4',
        ],
        output='screen',
    )

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

    spawn_object_2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', obj2_sdf,
            '-name', 'irregular_object_2',
            '-world', 'empty',
            '-x', '-0.15',
            '-y', '-0.10',
            '-z', '0.075',
            '-Y', '-0.8',
        ],
        output='screen',
    )

    bin_sdf = """<sdf version="1.6">
      <model name="drop_off_bin">
        <static>true</static>
        <pose>0 0 0 0 0 0</pose>
        <link name="bin_link">
          <visual name="bottom">
            <pose>0 0 0.01 0 0 0</pose>
            <geometry><box><size>0.24 0.18 0.02</size></box></geometry>
            <material><ambient>0.2 0.8 0.2 1</ambient><diffuse>0.2 0.8 0.2 1</diffuse></material>
          </visual>
          <collision name="bottom_col">
            <pose>0 0 0.01 0 0 0</pose>
            <geometry><box><size>0.24 0.18 0.02</size></box></geometry>
          </collision>
          <visual name="wall_front">
            <pose>0 0.09 0.05 0 0 0</pose>
            <geometry><box><size>0.24 0.02 0.08</size></box></geometry>
            <material><ambient>0.15 0.6 0.15 1</ambient><diffuse>0.15 0.6 0.15 1</diffuse></material>
          </visual>
          <visual name="wall_back">
            <pose>0 -0.09 0.05 0 0 0</pose>
            <geometry><box><size>0.24 0.02 0.08</size></box></geometry>
            <material><ambient>0.15 0.6 0.15 1</ambient><diffuse>0.15 0.6 0.15 1</diffuse></material>
          </visual>
          <visual name="wall_left">
            <pose>-0.12 0 0.05 0 0 0</pose>
            <geometry><box><size>0.02 0.18 0.08</size></box></geometry>
            <material><ambient>0.15 0.6 0.15 1</ambient><diffuse>0.15 0.6 0.15 1</diffuse></material>
          </visual>
          <visual name="wall_right">
            <pose>0.12 0 0.05 0 0 0</pose>
            <geometry><box><size>0.02 0.18 0.08</size></box></geometry>
            <material><ambient>0.15 0.6 0.15 1</ambient><diffuse>0.15 0.6 0.15 1</diffuse></material>
          </visual>
        </link>
      </model>
    </sdf>"""

    spawn_bin = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', bin_sdf,
            '-name', 'drop_off_bin',
            '-world', 'empty',
            '-x', '0.40',
            '-y', '-0.20',
            '-z', '0.05',
        ],
        output='screen',
    )

    # Parameter Bridge (Gazebo Sim -> ROS 2)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/birobot/depth_camera/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0',
            '--switch-timeout', '30.0',
        ],
        output='screen',
    )

    arm1_trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm1_joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0',
            '--switch-timeout', '30.0',
        ],
        output='screen',
    )

    arm2_trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm2_joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0',
            '--switch-timeout', '30.0',
        ],
        output='screen',
    )

    arm1_gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm1_gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0',
            '--switch-timeout', '30.0',
        ],
        output='screen',
    )

    arm2_gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm2_gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0',
            '--switch-timeout', '30.0',
        ],
        output='screen',
    )

    load_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(period=8.0, actions=[joint_state_broadcaster_spawner])
            ],
        )
    )

    sensors_3d_yaml = load_yaml('birobot_moveit_config', 'config/sensors_3d.yaml') or {}
    octomap_config = {
        'octomap_frame': 'world',
        'octomap_resolution': 0.05,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipelines_config,
            {'ompl': ompl_planning_yaml},
            trajectory_execution,
            moveit_controllers_yaml,
            sensors_3d_yaml,
            octomap_config,
            {'publish_robot_description_semantic': True},
            {'use_sim_time': True},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    load_arm_controllers_and_moveit = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                arm1_trajectory_spawner,
                arm2_trajectory_spawner,
                arm1_gripper_spawner,
                arm2_gripper_spawner,
                move_group_node,
                rviz_node,
            ],
        )
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo_sim,
        clock_bridge,
        spawn_entity,
        spawn_object_1,
        spawn_object_2,
        spawn_bin,
        robot_state_publisher,
        load_jsb,
        load_arm_controllers_and_moveit,
    ])
