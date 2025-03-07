from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('trajectory_visualization')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'trajectory_visualization.rviz')
    return LaunchDescription([
        Node(
            package='trajectory_visualization',
            executable='trajectory_publisher_saver',
            name='trajectory_publisher_saver'
        ),
        Node(
            package='trajectory_visualization',
            executable='trajectory_reader_publisher',
            name='trajectory_reader_publisher',
            parameters=[{'trajectory_file': 'trajectory.json'}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file]
        )
    ])
