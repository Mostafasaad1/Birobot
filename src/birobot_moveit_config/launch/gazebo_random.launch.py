"""
gazebo_random.launch.py — Birobot launch bringing up Gazebo Sim + MoveIt 2 + RViz2 + ros2_control + Workspace Objects
with dynamic randomized spawn for irregular_object_1 in the collaborative dual-arm & camera workspace,
and full sensor bridges for PointCloud2, RGB Image, Depth Image, and CameraInfo.
"""

import math
import os
import random
import shutil
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
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
    gz_gui_config_path = PathJoinSubstitution([
        birobot_moveit_share,
        'config',
        'gazebo_gui.config'
    ])

    # Declare launch arguments for spawn customization
    declared_args = [
        DeclareLaunchArgument(
            'randomize',
            default_value='true',
            description='Randomize red irregular_object_1 spawn within Arm 1 workspace',
        ),
        DeclareLaunchArgument(
            'zone',
            default_value='other_side',
            description='Spawn zone around Arm 1: "other_side" (past table border X in [-1.08, -0.90]), "front" (on table X in [-0.48, -0.15]), or "all"',
        ),
        DeclareLaunchArgument(
            'object_x',
            default_value='-0.35',
            description='Spawn X (m) for irregular_object_1 when randomize is false',
        ),
        DeclareLaunchArgument(
            'object_y',
            default_value='0.00',
            description='Spawn Y (m) for irregular_object_1 when randomize is false',
        ),
        DeclareLaunchArgument(
            'object_z',
            default_value='0.15',
            description='Spawn Z (m) for irregular_object_1',
        ),
        DeclareLaunchArgument(
            'object_yaw',
            default_value='0.00',
            description='Spawn Yaw (rad) for irregular_object_1 when randomize is false',
        ),
    ]

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
        'trajectory_execution.allowed_start_tolerance': 0.05,
    }

    rviz_config_file = os.path.join(birobot_moveit_share, 'config', 'moveit.rviz')

    # ── 2. Gazebo Sim & Spawners ──────────────────────────────────────────────
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r empty.sdf --gui-config ', gz_gui_config_path],
            'on_exit_shutdown': 'true',
        }.items(),
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
            <inertia><ixx>0.00193</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.00260</iyy><iyz>0</iyz><izz>0.00120</izz></inertia>
          </inertial>
          <visual name="visual">
            <geometry><box><size>0.15 0.08 0.20</size></box></geometry>
            <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
          </visual>
          <collision name="collision">
            <geometry><box><size>0.15 0.08 0.20</size></box></geometry>
          </collision>
        </link>
      </model>
    </sdf>"""

    def spawn_object_1_factory(context, *args, **kwargs):
        randomize_flag = context.launch_configurations.get('randomize', 'true').lower() in ('true', '1', 'yes')
        zone = context.launch_configurations.get('zone', 'other_side').lower()
        if randomize_flag:
            while True:
                if zone in ('other_side', 'otherside', 'rear', 'back'):
                    # Past table border behind Arm 1:
                    # Table ends at X = -0.800. Object footprint radius is ~0.085m.
                    # Center X in [-1.080, -0.900] guarantees object closest edge is <= -0.815m (100% off table).
                    x_val = random.uniform(-1.080, -0.900)
                    y_val = random.uniform(-0.250, 0.250)
                    dist_base = math.hypot(x_val - (-0.60), y_val)
                    if 0.24 <= dist_base <= 0.52:
                        break
                elif zone in ('front', 'infront'):
                    x_val = random.uniform(-0.480, -0.150)
                    y_val = random.uniform(-0.280, 0.280)
                    dist_base = math.hypot(x_val - (-0.60), y_val)
                    if 0.24 <= dist_base <= 0.52:
                        break
                else:  # all
                    subzone = 'other_side' if random.random() < 0.70 else 'front'
                    if subzone == 'other_side':
                        x_val = random.uniform(-1.080, -0.900)
                        y_val = random.uniform(-0.250, 0.250)
                    else:
                        x_val = random.uniform(-0.480, -0.150)
                        y_val = random.uniform(-0.280, 0.280)
                    dist_base = math.hypot(x_val - (-0.60), y_val)
                    if 0.24 <= dist_base <= 0.52:
                        break

            spawn_x = str(round(x_val, 3))
            spawn_y = str(round(y_val, 3))
            # Table top is at Z=0.05m (spawns at Z=0.15m). Past table border is ground at Z=0.00m (spawns at Z=0.10m)
            is_on_table = (-0.80 <= float(spawn_x) <= 0.80) and (-0.40 <= float(spawn_y) <= 0.40)
            spawn_z = '0.15' if is_on_table else '0.10'
            spawn_yaw = str(round(random.uniform(-1.5708, 1.5708), 3))
            dist_base = round(math.hypot(float(spawn_x) - (-0.60), float(spawn_y)), 3)
            angle_base = round(math.degrees(math.atan2(float(spawn_y), float(spawn_x) - (-0.60))), 1)
            side_label = "OTHER SIDE (PAST TABLE BORDER)" if not is_on_table else "FRONT (on table)"
            print(
                f"\n=======================================================\n"
                f"[GAZEBO RANDOM SPAWN] Spawned irregular_object_1 in Arm 1 workspace ({side_label}):\n"
                f"  X     = {spawn_x} m\n"
                f"  Y     = {spawn_y} m\n"
                f"  Z     = {spawn_z} m ({'ON TABLE' if is_on_table else 'GROUND (PAST TABLE BORDER)'})\n"
                f"  Yaw   = {spawn_yaw} rad ({math.degrees(float(spawn_yaw)):.1f} deg)\n"
                f"  Dist  = {dist_base} m from Arm 1 base\n"
                f"  Angle = {angle_base} deg relative to Arm 1\n"
                f"  Zone  = {zone}\n"
                f"=======================================================\n",
                flush=True
            )
        else:
            spawn_x = context.launch_configurations.get('object_x', '-0.35')
            spawn_y = context.launch_configurations.get('object_y', '0.00')
            spawn_z = context.launch_configurations.get('object_z', '0.15')
            spawn_yaw = context.launch_configurations.get('object_yaw', '0.00')
            print(
                f"\n[GAZEBO FIXED SPAWN] irregular_object_1 fixed pose: "
                f"x={spawn_x}, y={spawn_y}, z={spawn_z}, yaw={spawn_yaw}\n",
                flush=True
            )

        return [
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-string', obj1_sdf,
                    '-name', 'irregular_object_1',
                    '-world', 'empty',
                    '-x', spawn_x,
                    '-y', spawn_y,
                    '-z', spawn_z,
                    '-Y', spawn_yaw,
                ],
                output='screen',
            )
        ]

    spawn_object_1_action = OpaqueFunction(function=spawn_object_1_factory)

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
            '-x', '-0.10',
            '-y', '-0.22',
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

    # Parameter Bridge — clock and sensor data (PointCloud2, RGB Image, Depth Image, CameraInfo)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/birobot/depth_camera/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/birobot/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/birobot/depth_camera/points@sensor_msgs/msg/Image[gz.msgs.Image',
            '/birobot/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/birobot/depth_camera/points', '/birobot/depth_camera/depth_image'),
        ],
        output='screen'
    )

    # Dedicated bridge for attach/detach signals with TRANSIENT_LOCAL QoS.
    attach_detach_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/arm1/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/arm1/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/arm2/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/arm2/detach@std_msgs/msg/Empty]gz.msgs.Empty',
        ],
        parameters=[{
            'qos_overrides./arm1/attach.publisher.durability': 'transient_local',
            'qos_overrides./arm1/attach.publisher.reliability': 'reliable',
            'qos_overrides./arm1/attach.subscription.durability': 'transient_local',
            'qos_overrides./arm1/attach.subscription.reliability': 'reliable',
            'qos_overrides./arm1/detach.publisher.durability': 'transient_local',
            'qos_overrides./arm1/detach.publisher.reliability': 'reliable',
            'qos_overrides./arm1/detach.subscription.durability': 'transient_local',
            'qos_overrides./arm1/detach.subscription.reliability': 'reliable',
            'qos_overrides./arm2/attach.publisher.durability': 'transient_local',
            'qos_overrides./arm2/attach.publisher.reliability': 'reliable',
            'qos_overrides./arm2/attach.subscription.durability': 'transient_local',
            'qos_overrides./arm2/attach.subscription.reliability': 'reliable',
            'qos_overrides./arm2/detach.publisher.durability': 'transient_local',
            'qos_overrides./arm2/detach.publisher.reliability': 'reliable',
            'qos_overrides./arm2/detach.subscription.durability': 'transient_local',
            'qos_overrides./arm2/detach.subscription.reliability': 'reliable',
        }],
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

    return LaunchDescription(
        declared_args + [
            set_gz_resource_path,
            gazebo_sim,
            clock_bridge,
            attach_detach_bridge,
            spawn_entity,
            spawn_object_1_action,
            spawn_object_2,
            spawn_bin,
            robot_state_publisher,
            load_jsb,
            load_arm_controllers_and_moveit,
        ]
    )
