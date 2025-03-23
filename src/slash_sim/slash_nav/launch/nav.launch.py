from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    map_file = os.path.join(
        get_package_share_directory('slash_slam'),
        'map',
        'map.yaml'
    )
    
    nav_param_file = os.path.join(
        get_package_share_directory('slash_nav'),
        'config',
        'nav.yaml'
    )

    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 
        'launch'
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'True',
            'params_file': nav_param_file}.items(),
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join( get_package_share_directory('slash_nav'), 'rviz', 'navi.rviz')],
    )

    saye_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slash_gazebo'), 'launch','gazebo.launch.py')
        )
    )

    ld = LaunchDescription()
    ld.add_action(navigation_cmd)
    # ld.add_action(rviz_cmd)
    ld.add_action(saye_gazebo_cmd)
    return ld