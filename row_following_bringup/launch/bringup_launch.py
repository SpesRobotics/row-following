import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def create_image_compression_nodes(topic_names):
    nodes = []
    for topic_name in topic_names:
        node = Node(
            package='image_transport',
            executable='republish',
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', topic_name),
                ('out/compressed', topic_name + '/compressed'),
            ],
            output='screen',
        )
        nodes.append(node)
    return nodes

def generate_launch_description():
    # Get the directory path of spesbot_isaac
    package_dir = get_package_share_directory("spesbot_isaac")
    launch_file = os.path.join(package_dir, 'launch', 'isaac_launch.py')
    port = LaunchConfiguration('port', default='8765')
    address = LaunchConfiguration('address', default='192.168.2.168')

    declare_port_argument = DeclareLaunchArgument(
        'port',
        default_value='8765',
        description='Port to run the Foxglove Bridge server on'
    )

    declare_address_argument = DeclareLaunchArgument(
        'address',
        default_value='192.168.2.168',
        description='Address to run the Foxglove Bridge server on'
    )

    # Declare simulation argument
    declare_sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Launch simulation'
    )

    image_compression_nodes = create_image_compression_nodes([
        '/camera/robot/view',
        '/top/view/rgb',
    ])

    return LaunchDescription([
        declare_sim_arg,
        declare_port_argument,
        declare_address_argument,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'sim': LaunchConfiguration('sim')}.items()
        ),
        *image_compression_nodes,
        Node(
            package='row_following_bringup',
            executable='row_follow_bt.py',
            output='screen'
        ),
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
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            parameters=[{
                'port': port,
                'address': address,
            }],
            output='screen',
        )
    ])
