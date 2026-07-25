import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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

    controllers_yaml_path = '/tmp/birobot_controllers.yaml'
    if not os.path.exists(controllers_yaml_path):
        controllers_yaml_path = os.path.join(birobot_description_share, 'config', 'controllers.yaml')

    xacro_file = os.path.join(birobot_description_share, 'urdf', 'birobot.urdf.xacro')
    doc = xacro.process_file(
        xacro_file,
        mappings={
            'sim_ignition': 'true',
            'use_fake_hardware': 'false',
            'controllers_yaml': controllers_yaml_path,
        }
    )
    robot_description = {'robot_description': doc.toxml()}

    srdf_file = os.path.join(birobot_moveit_share, 'config', 'birobot.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_yaml = load_yaml('birobot_moveit_config', 'config/kinematics.yaml') or {}
    joint_limits_yaml = load_yaml('birobot_moveit_config', 'config/joint_limits.yaml') or {}
    ompl_planning_yaml = load_yaml('birobot_moveit_config', 'config/ompl_planning.yaml') or {}

    planning_pipelines_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl.planning_plugin': 'ompl_interface/OMPLPlanner',
    }

    node = Node(
        package='birobot_manipulation',
        executable='mtc_pick_place_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'robot_description_planning': joint_limits_yaml},
            planning_pipelines_config,
            {'ompl': ompl_planning_yaml},
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([node])
