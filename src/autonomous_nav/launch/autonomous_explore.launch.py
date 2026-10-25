from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('autonomous_nav')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation clock',
        ),

        # SLAM Toolbox (online async for mapping)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': os.path.join(
                    pkg_dir, 'config', 'slam_toolbox.yaml'
                ),
            }.items(),
        ),

        # Nav2 navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': os.path.join(
                    pkg_dir, 'config', 'nav2_params.yaml'
                ),
            }.items(),
        ),

        # Frontier explorer — autonomous exploration
        Node(
            package='autonomous_nav',
            executable='frontier_explorer',
            name='frontier_explorer',
            parameters=[{
                'use_sim_time': True,
                'min_frontier_size': 5,
                'exploration_timeout': 30.0,
                'blacklist_radius': 0.5,
                'recheck_interval': 1.0,
            }],
            output='screen',
        ),

        # RViz2 with custom config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'rviz_config.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
