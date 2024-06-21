import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    row_following_bringup_dir = get_package_share_directory('row_following_bringup')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(row_following_bringup_dir, 'launch', 'bringup_launch.py')
        )
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(row_following_bringup_dir, 'launch', 'nav2_bringup_launch.py')
        )
    )

    generate_path_node = Node(
        package='row_following_behavior',
        executable='generate_path',
        name='generate_path',
        output='screen'
    )

    behavior_node = Node(
        package='row_following_behavior',
        executable='behavior',
        name='behavior',
        output='screen'
    )

    return LaunchDescription([
        bringup_launch,
        nav2_bringup_launch,
        generate_path_node,
        behavior_node
    ])
