from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_localizer')
    rviz_config = os.path.join(pkg_share, 'rviz', 'localization_config.rviz')

    return LaunchDescription([
        Node(
            package='aruco_localizer',
            executable='stream_to_ros',
            name='rtsp_stream_publisher',
            output='screen'
        ),
        Node(
            package='aruco_localizer',
            executable='localize',
            name='aruco_localizer_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
