import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=os.path.expanduser('~/autoware/map'),
        description='Path to Autoware map directory containing map_projector_info.yaml'
    )

    # Create nodes for all executables
    autoware_vehicle_plugin_node = Node(
        package='autoware_cosim',
        executable='autoware_vehicle_plugin',
        name='autoware_vehicle_plugin',
        output='screen',
        parameters=[{
            'control_cav': False,
            'cosim_controlled_vehicle_keys': ['terasim_actor_info'],
            'map_path': LaunchConfiguration('map_path'),
        }]
    )

    autoware_tls_plugin_node = Node(
        package='autoware_cosim',
        executable='autoware_tls_plugin',
        name='autoware_tls_plugin',
        output='screen'
    )

    autoware_dummy_grid_node = Node(
        package='autoware_cosim',
        executable='autoware_dummy_grid',
        name='autoware_dummy_grid',
        output='screen'
    )

    autoware_vehicle_report_node = Node(
        package='autoware_cosim',
        executable='autoware_vehicle_report',
        name='autoware_vehicle_report',
        output='screen'
    )

    autoware_planning_node = Node(
        package='autoware_cosim',
        executable='autoware_planning',
        name='autoware_planning',
        output='screen'
    )

    return LaunchDescription([
        map_path_arg,
        autoware_vehicle_plugin_node,
        autoware_vehicle_report_node,
        autoware_tls_plugin_node,
        autoware_dummy_grid_node,
        autoware_planning_node,
    ]) 
