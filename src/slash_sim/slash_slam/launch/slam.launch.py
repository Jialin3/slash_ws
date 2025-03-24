import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    slam_params = {
        "use_sim_time": True,
        "base_frame": "saye",
        "odom_frame": "odom",
        "map_frame": "map"
    }
    slam_cmd = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        parameters=[slam_params]
    )

    robot_drive_cmd = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering'
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join( get_package_share_directory('slash_slam'), 'rviz', 'slam.rviz')],
    )

   

    ld = LaunchDescription()
    ld.add_action(slam_cmd)
    ld.add_action(robot_drive_cmd)
    ld.add_action(rviz_cmd)
    return ld