import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_manipulation = get_package_share_directory('birobot_manipulation')
    bt_xml = os.path.join(pkg_manipulation, 'config', 'bt_trees', 'auto_pick_place.xml')

    auto_pick_coordinator_node = Node(
        package='birobot_manipulation',
        executable='auto_pick_coordinator',
        name='auto_pick_coordinator',
        output='screen',
        parameters=[
            {'bt_xml_file': bt_xml}
        ]
    )

    return LaunchDescription([
        auto_pick_coordinator_node
    ])
