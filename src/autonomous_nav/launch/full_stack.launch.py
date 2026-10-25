from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('autonomous_nav')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_method = LaunchConfiguration('slam_method')
    enable_exploration = LaunchConfiguration('enable_exploration')
    enable_metrics = LaunchConfiguration('enable_metrics')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
        ),
        DeclareLaunchArgument(
            'slam_method', default_value='slam_toolbox',
            description='SLAM method: slam_toolbox or orb_slam3',
        ),
        DeclareLaunchArgument(
            'enable_exploration', default_value='true',
            description='Launch frontier explorer',
        ),
        DeclareLaunchArgument(
            'enable_metrics', default_value='true',
            description='Launch metrics collection',
        ),

        # SLAM Toolbox (default)
        GroupAction(
            condition=IfCondition(EqualsSubstitution(slam_method, 'slam_toolbox')),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(slam_toolbox_dir, 'launch',
                                     'online_async_launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'slam_params_file': os.path.join(
                            pkg_dir, 'config', 'slam_toolbox.yaml'
                        ),
                    }.items(),
                ),
            ],
        ),

        # ORB-SLAM3 visual SLAM (alternative)
        GroupAction(
            condition=IfCondition(EqualsSubstitution(slam_method, 'orb_slam3')),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_dir, 'launch', 'orb_slam3.launch.py')
                    ),
                    launch_arguments={'use_sim_time': use_sim_time}.items(),
                ),
            ],
        ),

        # Nav2 stack
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

        # Frontier explorer
        Node(
            condition=IfCondition(enable_exploration),
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

        # Metrics collection
        Node(
            condition=IfCondition(enable_metrics),
            package='autonomous_nav',
            executable='exploration_metrics',
            name='exploration_metrics',
            parameters=[{
                'use_sim_time': True,
                'output_dir': os.path.join(pkg_dir, 'metrics_output'),
                'sample_interval': 2.0,
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
