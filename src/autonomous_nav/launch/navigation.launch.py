from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('autonomous_nav')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('nav2_params')
    run_mission = LaunchConfiguration('run_mission')
    mission_file = LaunchConfiguration('mission_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_dir, 'maps', 'warehouse.yaml'),
            description='Path to map yaml file',
        ),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
            description='Nav2 parameter file',
        ),
        DeclareLaunchArgument(
            'run_mission',
            default_value='false',
            description='Start the waypoint mission once Nav2 is up',
        ),
        DeclareLaunchArgument(
            'mission_file',
            default_value=os.path.join(
                pkg_dir, 'config', 'warehouse_mission.yaml'
            ),
            description='Waypoint mission YAML',
        ),

        # Launch Nav2 bringup with AMCL localization and saved map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_file,
                'params_file': nav2_params,
            }.items(),
        ),

        # Multi-waypoint mission executor
        Node(
            condition=IfCondition(run_mission),
            package='autonomous_nav',
            executable='waypoint_manager',
            name='waypoint_manager',
            parameters=[{
                'use_sim_time': True,
                'waypoints_file': mission_file,
            }],
            output='screen',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'rviz_config.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
