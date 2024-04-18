from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    randomize_node = Node(
        package='row_following_py_pkg',
        executable='randomize_node',
        name='randomize_node'
    )

    image_subscriber_node = Node(
        package="row_following_py_pkg",
        executable="image_subscriber",
        name="image_subscriber_node"
        # output='screen'
    )

    # semantic_image_subscriber_node = Node(
    #     package="row_following_py_pkg",
    #     executable="semantic_image_subscriber",
    #     name="semantic_image_subscriber_node"
    #     # output='screen'
    # )

    ld.add_action(randomize_node)
    ld.add_action(image_subscriber_node)
    # ld.add_action(semantic_image_subscriber_node)

    return ld
