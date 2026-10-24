"""Gazebo Harmonic + TurtleBot3 in the custom warehouse world.

The robot spawns at the world origin so SLAM's map frame lines up with the
coordinates used in config/warehouse_mission.yaml.
"""
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('autonomous_nav')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    tb3_launch_dir = os.path.join(tb3_gazebo, 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    gui = LaunchConfiguration('gui')
    world = os.path.join(pkg_dir, 'worlds', 'warehouse.world')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Open the Gazebo GUI client',
        ),

        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.path.join(tb3_gazebo, 'models')
        ),

        # Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-r -s -v2 ', world],
                'on_exit_shutdown': 'true',
            }.items(),
        ),

        # Gazebo GUI
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': '-g -v2 ',
                'on_exit_shutdown': 'true',
            }.items(),
            condition=IfCondition(gui),
        ),

        # Robot description + spawn + ROS<->GZ bridge from turtlebot3_gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
        ),
    ])
