# License: Apache License 2.0
# https://github.com/DynoRobotics/dyno-licence/raw/master/LICENCE_APACHE_SAM
# Author: Samuel Lindgren

"""Brings up nodes for Shelfbot"""

from launch import LaunchDescription
import launch.actions
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    map_yaml_file = launch.substitutions.LaunchConfiguration('map')

    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='false')

    params_file = launch.substitutions.LaunchConfiguration(
        'params', default=[launch.substitutions.ThisLaunchFileDir(), '/nav2_params.yaml'])

    urdf_file_name = get_package_share_directory(
        'turtlebot3_description') + '/urdf/turtlebot3_burger.urdf'

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'map', default_value=[launch.substitutions.ThisLaunchFileDir(), '/test_map.yaml'], description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation clock if true'),

        launch_ros.actions.Node(
            package='nav2_map_server',
            node_executable='map_server',
            node_name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, {'yaml_filename': map_yaml_file}]),

        launch_ros.actions.Node(
            package='nav2_world_model',
            node_executable='world_model',
            output='screen',
            parameters=[params_file]),

        launch_ros.actions.Node(
            package='nav2_simple_navigator',
            node_executable='simple_navigator',
            node_name='simple_navigator',
            output='screen'),

        # launch_ros.actions.Node(
        #     package='nav2_bt_navigator',
        #     node_executable='bt_navigator',
        #     node_name='bt_navigator',
        #     output='screen'),

        # launch_ros.actions.Node(
        #     package='nav2_amcl',
        #     node_executable='amcl',
        #     node_name='amcl',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}]),

        launch_ros.actions.Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='static_map_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '1', '/map', '/odom']),

        launch_ros.actions.Node(
            package='dwb_controller',
            node_executable='dwb_controller',
            output='screen',
            parameters=[params_file]),

        launch_ros.actions.Node(
            package='nav2_navfn_planner',
            node_executable='navfn_planner',
            node_name='navfn_planner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),

        launch_ros.actions.Node(
            package='nav2_mission_executor',
            node_executable='mission_executor',
            node_name='mission_executor',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),

        launch_ros.actions.Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_file_name]),

    ])
