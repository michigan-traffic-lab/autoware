import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([

        ############################################################
        # Autoware Interface to handle the simulation requests
        ############################################################
        Node(
            package='mcity_demo',
            namespace='/mcity',
            executable='autoware_interface_demo_cosim',
        ),

        ############################################################
        # Cosim
        ############################################################
        Node(
            package='autoware_cosim',
            namespace='/mcity',
            executable='autoware_vehicle_plugin',
            parameters=[
                {'control_cav': True},
                {'cosim_controlled_vehicle_keys': ["terasim_actor_info"]}
            ],
        ),
        Node(
            package='autoware_cosim',
            namespace='/mcity',
            executable='autoware_tls_plugin',
        ),
        Node(
            package='autoware_cosim',
            namespace='/mcity',
            executable='autoware_dummy_grid',
        ),
    ])
