import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    slam_params = {
        "use_sim_time": True,
        "base_frame": "slash",
        "odom_frame": "odom",
        "map_frame": "map"
    }
    slam_cmd = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        parameters=[slam_params]
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
    )

    ld = LaunchDescription()
    ld.add_action(slam_cmd)
    ld.add_action(rviz_cmd)
    return ld