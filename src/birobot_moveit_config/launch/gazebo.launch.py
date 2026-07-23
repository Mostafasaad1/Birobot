"""
gazebo.launch.py — Birobot launch bringing up Gazebo Sim + MoveIt 2 + RViz2 + ros2_control

Launches:
  1. GZ_SIM_RESOURCE_PATH env var (ensures Gazebo Sim resolves ur10e meshes)
  2. Gazebo Sim empty world (ros_gz_sim)
  3. birobot spawn entity in Gazebo Sim
  4. robot_state_publisher
  5. ros2_control_node + controller spawners
  6. move_group
  7. rviz2
"""

import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
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
    # IMPORTANT: process xacro ONCE and reuse for BOTH robot_state_publisher
    # AND Gazebo spawn. Using two different xacro invocations (e.g. via
    # subprocess with different args) causes RSP and Gazebo to have divergent
    # robot models, which is the root cause of the Gazebo↔RViz pose mismatch.
    original_controllers_yaml_path = os.path.join(
        birobot_description_share, 'config', 'controllers.yaml'
    )
    # WORKAROUND for ROS 2 Jazzy controller_manager bug: 
    # controller_manager copies node arguments to controllers, but blindly drops any argument 
    # containing the substring "robot_description". We copy the yaml to /tmp to avoid the substring.
    controllers_yaml_path = '/tmp/birobot_controllers.yaml'
    import shutil
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

    # Write the canonical URDF to a temp file for Gazebo spawn_entity
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
        '/**': {'ros__parameters': {'robot_description': robot_description_str}},
    }
    if controllers_config:
        merged_ros2ctrl_params.update(controllers_config)

    _ctrl_params_tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix='birobot_ros2ctrl_'
    )
    yaml.dump(merged_ros2ctrl_params, _ctrl_params_tmp)
    _ctrl_params_tmp.close()
    merged_ros2ctrl_params_file = _ctrl_params_tmp.name

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

    # =========================================================================
    # Launch Actions & Nodes
    # =========================================================================

    controllers_yaml_path = os.path.join(
        birobot_description_share, 'config', 'controllers.yaml'
    )

    # Gazebo Sim
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Spawn entity into Gazebo — use the same URDF as robot_state_publisher
    # (urdf_spawn_file was written from the xacro.process_file() call above)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_spawn_file,
            '-name', 'birobot',
            '-world', 'empty',
            '-allow_renaming', 'true',
        ],
        output='screen',
    )

    # Clock Bridge (Gazebo Sim -> ROS 2)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # (Removed static joint_state_publisher to prevent /joint_states conflicts with joint_state_broadcaster)


    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    arm1_trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm1_joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    arm2_trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm2_joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # Sequencing:
    #   spawn_entity exits  →  5s timer  →  JSB spawner
    #   JSB spawner exits   →  arm controller spawners
    # Once JSB is active it publishes real /joint_states which overrides JSP.
    load_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner])
            ],
        )
    )

    load_arm_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm1_trajectory_spawner, arm2_trajectory_spawner],
        )
    )

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

    return LaunchDescription([
        set_gz_resource_path,
        gazebo_sim,
        clock_bridge,
        spawn_entity,
        robot_state_publisher,
        load_jsb,
        load_arm_controllers,
        move_group_node,
        rviz_node,
    ])
