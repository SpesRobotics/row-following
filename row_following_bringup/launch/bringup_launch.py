import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the directory path of spesbot_isaac
    package_dir = get_package_share_directory("spesbot_isaac")
    launch_file = os.path.join(package_dir, 'launch', 'isaac_launch.py')

    # Declare simulation argument
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Launch simulation'
    )

    return LaunchDescription([
        sim_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'sim': LaunchConfiguration('sim')}.items()
        ),
        # Node(
        #     package='row_following_bringup',
        #     executable='row_follow.py',
        #     output='screen'
        # ),
        Node(
            package='spesbot_isaac',
            executable='tf2navsatfix.py',
            output='screen'
        ),
        Node(
            package='spesbot_isaac',
            executable='navsatfix2tf.py',
            output='screen'
        ),
        # Node(
        #     package='row_following_bringup',
        #     executable='nav2_test.py',
        #     output='screen'
        # )
    ])
