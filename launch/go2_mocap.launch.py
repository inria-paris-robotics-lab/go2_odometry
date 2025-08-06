from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    minimal_state_publisher_launch_file = PathJoinSubstitution(
        [FindPackageShare("go2_odometry"), "launch", "go2_state_publisher.launch.py"]
    )

    mimic_go2_odometry_arg = DeclareLaunchArgument(
        "mimic_go2_odometry",
        default_value=TextSubstitution(text="1"),
        description="Types of use of the motion capture: 0 (as a ground truth), 1 (as a perfect pose estimator) \n\t\tIf 0 is selected will publish on /tf odom -> base_mocap \n\t\tIf 1 is selected will publish on /tf odom -> base as well as /odometry/filtered",
    )

    mocap_base_frame_arg = DeclareLaunchArgument(
        "base_frame",
        default_value="base",
        description="[IF MOCAP] Name of the base frame (fixed to robot) of the mocap.",
    )
    mocap_odom_frame_arg = DeclareLaunchArgument(
        "odom_frame",
        default_value="odom",
        description="[IF MOCAP] Name of the odom frame (fixed to world) of the mocap.",
    )
    mocap_object_name_arg = DeclareLaunchArgument(
        "wanted_body", default_value="Go2", description="[IF MOCAP] Name of the object tracked by the motion capture."
    )
    mocap_ip_arg = DeclareLaunchArgument(
        "qualisys_ip",
        default_value="192.168.75.4",
        description="[IF MOCAP] IP used by the motion capture to publish data.",
    )
    mocap_publishing_freq_arg = DeclareLaunchArgument(
        "publishing_freq",
        default_value=TextSubstitution(text="110"),
        description="[IF MOCAP] Publishing frequency of the odom to robot base transform. Max limit of 300Hz.",
    )

    return LaunchDescription(
        [
            mocap_base_frame_arg,
            mocap_odom_frame_arg,
            mocap_object_name_arg,
            mocap_ip_arg,
            mocap_publishing_freq_arg,
            mimic_go2_odometry_arg,
            Node(
                package="go2_odometry",
                executable="mocap_base_pose.py",
                name="mocap_base_estimator",
                parameters=[
                    {
                        "base_frame": LaunchConfiguration("base_frame"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "wanted_body": LaunchConfiguration("wanted_body"),
                        "qualisys_ip": LaunchConfiguration("qualisys_ip"),
                        "publishing_freq": LaunchConfiguration("publishing_freq"),
                        "mimic_go2_odometry": LaunchConfiguration("mimic_go2_odometry"),
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([minimal_state_publisher_launch_file]),
                condition=IfCondition(LaunchConfiguration("mimic_go2_odometry")),
            ),
        ]
    )
