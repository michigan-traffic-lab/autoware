import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    http_host_arg = DeclareLaunchArgument(
        'http_host',
        default_value='localhost',
        description='HTTP server host for TeraSim co-simulation'
    )

    http_port_arg = DeclareLaunchArgument(
        'http_port',
        default_value='8000',
        description='HTTP server port for TeraSim co-simulation'
    )

    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=os.path.expanduser('~/Projects/autoware/map/zinger2sigma'),
        description='Path to Autoware map directory containing map_projector_info.yaml'
    )

    # Create nodes for all executables
    autoware_vehicle_plugin_http_node = Node(
        package='autoware_cosim',
        executable='autoware_vehicle_plugin_http',
        name='autoware_vehicle_plugin_http',
        output='screen',
        parameters=[{
            'control_cav': True,
            'http_host': LaunchConfiguration('http_host'),
            'http_port': LaunchConfiguration('http_port'),
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

    return LaunchDescription([
        http_host_arg,
        http_port_arg,
        map_path_arg,
        autoware_vehicle_plugin_http_node,
        autoware_tls_plugin_node,
        autoware_dummy_grid_node,
    ])
