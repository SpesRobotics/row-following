import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    navigation_dir = get_package_share_directory('row_following_bringup')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator']
    remappings = [('/tf', ['tf']),
                  ('/tf_static', ['tf_static'])]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (IsaacSim) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            navigation_dir, 'resource', 'nav2params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # TODO: Switch to warn later
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    load_composable_nodes = ComposableNodeContainer(
        namespace=namespace,
        remappings=remappings,
        parameters=[params_file],
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', log_level],
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                namespace=namespace,
                parameters=[
                    params_file
                ],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                namespace=namespace,
                parameters=[
                    params_file
                ],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                namespace=namespace,
                parameters=[
                    params_file
                ],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                namespace=namespace,
                parameters=[
                    params_file,
                ],
                remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                namespace=namespace,
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': True,
                             'node_names': lifecycle_nodes}])
        ],
    )

    return LaunchDescription([
        stdout_linebuf_envvar,
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_log_level_cmd,
        load_composable_nodes
    ])