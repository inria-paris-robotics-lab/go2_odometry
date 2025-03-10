from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():

    odom_type_arg = DeclareLaunchArgument(
        'odom_type',
        default_value='use_full_odom',
        description='Type of odometry desired between : fake, mocap, preint_ekf'
    )

    fake_odom_base_height_arg = DeclareLaunchArgument(
                            'base_height',
                            default_value=TextSubstitution(text='0.30'),
                            description='[IF FAKE ODOM] Height of the robot base.'
    )

    mocap_base_frame_arg = DeclareLaunchArgument(
                            'base_frame',
                            default_value='base',
                            description='[IF MOCAP] Name of the base frame (fixed to robot) of the mocap.'
    )
    mocap_odom_frame_arg = DeclareLaunchArgument(
                            'odom_frame',
                            default_value='odom',
                            description='[IF MOCAP] Name of the odom frame (fixed to world) of the mocap.'
    )
    mocap_object_name_arg = DeclareLaunchArgument(
                            'wanted_body',
                            default_value='Go2',
                            description='[IF MOCAP] Name of the object tracked by the motion capture.'
    )
    mocap_ip_arg = DeclareLaunchArgument(
                    'qualisys_ip',
                    default_value="192.168.75.2",
                    description='[IF MOCAP] IP used by the motion capture to publish data.'
    )
    mocap_publishing_freq_arg = DeclareLaunchArgument(
                                'publishing_freq',
                                default_value=TextSubstitution(text='110'),
                                description='[IF MOCAP] Publishing frequency of the odom to robot base transform. Max limit of 300Hz.'
    )


# launchfiles
    mocap_launch_file = PathJoinSubstitution([
                        FindPackageShare('go2_odometry'),
                        'launch',
                        'go2_mocap.launch.py'
                        ])
    
    fake_odom_launch_file = PathJoinSubstitution([
                            FindPackageShare('go2_odometry'),
                            'launch',
                            'go2_fake_odom.launch.py'
                            ])

    full_state_publisher_launch_file = PathJoinSubstitution([
                                    FindPackageShare('go2_odometry'),
                                    'launch',
                                    'go2_full_odometry.launch.py'
                                  ])

    return LaunchDescription([
        mocap_base_frame_arg,
        mocap_odom_frame_arg,
        mocap_object_name_arg,
        mocap_ip_arg,
        mocap_publishing_freq_arg,
        fake_odom_base_height_arg,

        odom_type_arg,

        IncludeLaunchDescription
        (
            PythonLaunchDescriptionSource([mocap_launch_file]),
            condition=IfCondition(PythonExpression(["'",LaunchConfiguration('odom_type'),"' == 'mocap'"]))
        ),

        IncludeLaunchDescription
        (
            PythonLaunchDescriptionSource([fake_odom_launch_file]),
            condition=IfCondition(PythonExpression(["'",LaunchConfiguration('odom_type'),"' == 'fake'"]))
        ),

<<<<<<< HEAD
        IncludeLaunchDescription
        (
=======
        Node(
            package="go2_odometry",
            executable="mocap_base_pose.py",
            name='mocap_base_estimator',
            condition=IfCondition(PythonExpression(["'",LaunchConfiguration('odom_type'),"' == 'mocap'"]))   
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([minimal_state_publisher_launch_file]),
            condition=IfCondition(PythonExpression(["'",LaunchConfiguration('odom_type'),"' == 'mocap'"]))  
        ),

        Node(
            package="go2_odometry",
            executable="imu_preint_node",
            name="imu_preint_node",
            condition=IfCondition(PythonExpression(["'",LaunchConfiguration('odom_type'),"' == 'preint_ekf'"]))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([full_state_publisher_launch_file]),
            condition=IfCondition(PythonExpression(["'",LaunchConfiguration('odom_type'),"' == 'preint_ekf'"]))
        ),

        IncludeLaunchDescription(
>>>>>>> 4c772c1 (wip)
            PythonLaunchDescriptionSource([full_state_publisher_launch_file]),
            condition=IfCondition(PythonExpression(["'",LaunchConfiguration('odom_type'),"' == 'use_full_odom'"]))
        )

])