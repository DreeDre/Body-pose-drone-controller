from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sjtu_launch_path = os.path.join(
        get_package_share_directory('sjtu_drone_bringup'),
        'launch',
        'sjtu_drone_gazebo.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sjtu_launch_path)
        ),
        Node(
            package='pose_estimator',
            executable='pose_publisher',
            output='screen'
        ),
        Node(
            package='gesture_controller',
            executable='gesture_controller',
            output='screen'
        ),
    ])
