from launch import LaunchDescription
from launch_ros.actions import Node
from unitree_description import GO2_DESCRIPTION_URDF_PATH


def generate_launch_description():
    # Read go2 urdf from unitree_description
    with open(GO2_DESCRIPTION_URDF_PATH, "r") as info:
        robot_desc = info.read()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc}],
                arguments=[GO2_DESCRIPTION_URDF_PATH],
            ),
            Node(
                package="go2_odometry",
                executable="state_converter_node",
                name="state_converter_node",
                parameters=[],
                output="screen",
            ),
        ]
    )
