import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def load_file(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(pkg_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(pkg_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # 1. URDF parameter
    birobot_description_share = get_package_share_directory('birobot_description')
    xacro_file = os.path.join(birobot_description_share, 'urdf', 'birobot.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    # 2. SRDF parameter
    robot_description_semantic_config = load_file('birobot_moveit_config', 'config/birobot.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # 3. Kinematics yaml
    kinematics_yaml = load_yaml('birobot_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml} if kinematics_yaml else {}

    # 4. Joint limits yaml
    joint_limits_yaml = load_yaml('birobot_moveit_config', 'config/joint_limits.yaml')
    robot_description_planning = {'robot_description_planning': joint_limits_yaml} if joint_limits_yaml else {}

    # 5. OMPL planning yaml (optional/default)
    ompl_planning_yaml = load_yaml('birobot_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning = {'ompl': ompl_planning_yaml} if ompl_planning_yaml else {}

    # 6. Planning pipelines config
    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
    }

    # RViz config path
    rviz_config_file = os.path.join(
        get_package_share_directory('birobot_moveit_config'), 'config', 'moveit.rviz'
    )

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[robot_description],
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
            planning_pipelines,
            ompl_planning,
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
        joint_state_publisher_gui,
        move_group_node,
        rviz_node,
        motion_planning_monitor_node,
    ])
