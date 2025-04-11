from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    state_publisher_launch_file = PathJoinSubstitution([
                                    FindPackageShare('go2_odometry'),
                                    'launch',
                                    'go2_state_publisher.launch.py'
                                  ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([state_publisher_launch_file])
        ),
        Node(
            package='go2_odometry',
            executable='dumb_odom.py',
            name='dumb_odom',
            output='screen',
            parameters=[],
           ),
])
