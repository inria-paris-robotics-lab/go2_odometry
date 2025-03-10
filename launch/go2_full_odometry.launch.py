from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    odom_type_arg = DeclareLaunchArgument(
        'odom_type',
        default_value='use_full_odom',
        description='Type of odometry desired between : fake, mocap, preint_ekf'
    )

    ekf_config_file             = PathJoinSubstitution([
                                    FindPackageShare("go2_odometry"),
                                    'config',
                                    'go2_ekf.yaml'
                                  ])
    
    preint_ekf_config_file        = PathJoinSubstitution([
                                    FindPackageShare("go2_odometry"),
                                    'config',
                                    'go2_ekf_w_preint.yaml'
                                  ])
    
    state_publisher_launch_file = PathJoinSubstitution([
                                    FindPackageShare('go2_odometry'),
                                    'launch',
                                    'go2_state_publisher.launch.py'
                                  ])

    return LaunchDescription([
        odom_type_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([state_publisher_launch_file])
           ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_file],
            condition=IfCondition(PythonExpression(["'",LaunchConfiguration('odom_type'), "' == 'use_full_odom'"])),
           ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[preint_ekf_config_file],
            condition=IfCondition(PythonExpression(["'",LaunchConfiguration('odom_type'), "' == 'preint_ekf'"])),
           ),

        Node(
            package='go2_odometry',
            executable='feet_to_odom.py',
            name='feet_to_odom',
            output='screen',
            parameters=[],
           ),
])
