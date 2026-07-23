"""
demo.launch.py — Birobot MoveIt 2 demo launch

The key architectural insight that makes ros2_control_node + spawners work:

  ros2_control_node gets parameters=[file_path, file_path] — only file paths,
  never dicts. This prevents the ROS 2 launch system from creating a global
  tmp params file (/tmp/launch_params_XXXX) that leaks a trailing --params-file
  flag into controller spawner processes.

  The robot_description URDF string is written to a temp YAML file at launch
  time and passed as a path string, not a dict.

Node startup sequence:
  1. robot_state_publisher   — TF broadcaster
  2. ros2_control_node       — mock_components/GenericSystem
                               initial_value: shoulder_lift=-1.57 (arms upright)
  3. → joint_state_broadcaster spawner  (publishes /joint_states)
  4. → arm1/arm2 trajectory controller spawners
  5. move_group / rviz2 / motion_planning_monitor (parallel)

dual_arms group shows TWO goal markers — correct. One per arm end effector
(arm1_tool0, arm2_tool0). They appear separated at the correct home position.
"""

import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
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

    # ── 1. Generate URDF ─────────────────────────────────────────────────────
    xacro_file = os.path.join(birobot_description_share, 'urdf', 'birobot.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description_str = doc.toxml()
    robot_description = {'robot_description': robot_description_str}

    # ── Write MERGED temp YAML for ros2_control_node ─────────────────────────
    # ros2_control_node propagates ALL --params-file entries to spawned controller
    # subprocesses. With two entries [file1, file2], controllers see:
    #   --params-file file1 --params-file file2
    # which triggers a Jazzy controller_manager parsing bug.
    # Solution: merge robot_description + controllers into ONE file.
    controllers_yaml_path = os.path.join(
        birobot_description_share, 'config', 'controllers.yaml'
    )
    with open(controllers_yaml_path, 'r') as f:
        controllers_config = yaml.safe_load(f)


    merged_ros2ctrl_params = {
        '/**': {'ros__parameters': {'robot_description': robot_description_str}},
    }
    # Merge controller_manager and controller params at top level
    if controllers_config:
        merged_ros2ctrl_params.update(controllers_config)

    _ctrl_params_tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix='birobot_ros2ctrl_'
    )
    yaml.dump(merged_ros2ctrl_params, _ctrl_params_tmp)
    _ctrl_params_tmp.close()
    merged_ros2ctrl_params_file = _ctrl_params_tmp.name

    # ── 2. SRDF ───────────────────────────────────────────────────────────────
    srdf_file = os.path.join(birobot_moveit_share, 'config', 'birobot.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_config = f.read()
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    # ── 3. Kinematics ─────────────────────────────────────────────────────────
    kinematics_yaml = load_yaml('birobot_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = (
        {'robot_description_kinematics': kinematics_yaml} if kinematics_yaml else {}
    )

    # ── 4. Joint limits ───────────────────────────────────────────────────────
    joint_limits_yaml = load_yaml('birobot_moveit_config', 'config/joint_limits.yaml')
    robot_description_planning = (
        {'robot_description_planning': joint_limits_yaml} if joint_limits_yaml else {}
    )

    # ── 5. OMPL planning pipelines ───────────────────────────────────────────
    ompl_planning_yaml = load_yaml('birobot_moveit_config', 'config/ompl_planning.yaml') or {}
    planning_pipelines_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
    }

    # ── 6. MoveIt controller config ───────────────────────────────────────────
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
    # Nodes
    # =========================================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ros2_control_node — ONE merged params file only.
    # Controllers spawned by controller_manager inherit --params-file args;
    # a single file prevents the double-flag bug in Jazzy.
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[merged_ros2ctrl_params_file],
        output='screen',
    )

    # joint_state_broadcaster — no per-controller params needed
    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # Arm trajectory controllers — need controllers_yaml for joint list
    arm1_trajectory_spawner = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'arm1_joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
            '-p', controllers_yaml_path,
        ],
        output='screen',
    )

    arm2_trajectory_spawner = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'arm2_joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
            '-p', controllers_yaml_path,
        ],
        output='screen',
    )

    # Proper startup sequencing
    load_jsb = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                TimerAction(period=1.5, actions=[joint_state_broadcaster_spawner])
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
        ],
    )

    motion_planning_monitor_node = Node(
        package='birobot_moveit_config',
        executable='motion_planning_monitor',
        output='screen',
        parameters=[
            {'arm_1_loaded': True},
            {'arm_2_loaded': True},
            {'dual_arms_loaded': True},
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        load_jsb,
        load_arm_controllers,
        move_group_node,
        rviz_node,
        motion_planning_monitor_node,
    ])
