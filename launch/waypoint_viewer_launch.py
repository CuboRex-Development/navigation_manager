from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

import os

# ここに読み込むwaypointファイルのファイル名を記入
# pkg/waypoints内のcsvファイルから読み込みます。
waypoint_file = 'waypoints.csv'
datum = [35.72514598638518, 139.8682489998428,]

# waypointファイルを読み込むための文字列処理
simple_commander_share_dir = get_package_share_directory('navigation_manager')
filenames = [simple_commander_share_dir + '/' + waypoint_file]
rviz_file = os.path.join(get_package_share_directory('navigation_manager'), 'rviz', 'waypoint_viewer.rviz')
# filenames = waypoint_file


def generate_launch_description():
    map_yaml_file = os.path.join(FindPackageShare(package='package_name').find('cugo_ros2_control'), 'map', 'map.yaml')
    lifecycle_nodes = ['map_server']

    print("waypoint : " + filenames[0])
    print("map : " + map_yaml_file)
    print("rviz : " + rviz_file)
    
    
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            output='screen'
        ),
        Node(
            package='navigation_manager',
            executable='waypoint_viewer',
            name='waypoint_viewer',
            parameters=[{'filename_list': filenames, 'datum_position': datum}],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': map_yaml_file}
            ]
            
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': lifecycle_nodes}]
        ),
    ])

