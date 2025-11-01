from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get paths
    pkg_path = get_package_share_directory('bot_trajectory')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'ink_view.rviz')  # optional RViz config

    # Get TurtleBot3 model (waffle)
    turtlebot3_model = 'waffle'

    return LaunchDescription([
        # Launch Gazebo (empty world)
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn TurtleBot3 Waffle at (x=0, y=0, z=0)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3_waffle',
                '-database', f'turtlebot3_{turtlebot3_model}',
                '-x', '0.0', '-y', '0.0', '-z', '0.01'
            ],
            output='screen'
        ),

        # Run your ink trail node
        Node(
            package='bot_trajectory',
            executable='path_marker',
            name='path_marker',
            output='screen'
        ),

        # Launch RViz2
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        ),
    ])
