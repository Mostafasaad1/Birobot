import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_birobot_description = get_package_share_directory('birobot_description')
    pkg_birobot_moveit = get_package_share_directory('birobot_moveit_config')
    pkg_birobot_manipulation = get_package_share_directory('birobot_manipulation')

    spawn_objects = LaunchConfiguration('spawn_objects')
    use_rviz = LaunchConfiguration('use_rviz')

    declared_arguments = [
        DeclareLaunchArgument(
            'spawn_objects',
            default_value='true',
            description='Spawn irregular test objects in Gazebo Sim workcell',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 visualization with MoveIt motion planning panel',
        ),
    ]

    # 1. Gazebo Sim Workcell (Simulation + ros2_control hardware controllers + RSP + test objects + RViz2)
    simulation_workcell = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_birobot_description, 'launch', 'simulation_workcell.launch.py')
        ),
        launch_arguments={
            'spawn_objects': spawn_objects,
            'use_rviz': use_rviz,
        }.items(),
    )

    # 2. MoveIt 2 MoveGroup Node (OMPL, SRDF, Kinematics, 3D Perception & get_planning_scene service)
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_birobot_moveit, 'launch', 'move_group.launch.py')
        )
    )

    # 3. MTC Pick and Place Lifecycle Node
    pick_place_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_birobot_manipulation, 'launch', 'pick_place.launch.py')
        )
    )

    return LaunchDescription(
        declared_arguments + [
            simulation_workcell,
            move_group,
            pick_place_node,
        ]
    )
