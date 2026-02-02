"""
TeraSim-Autoware Co-simulation Launch File

Launches TeraSim with run_experiments.py --autoware and the vehicle plugin.
SUMO runs at real-time (no tick driver needed).

Usage:
    ros2 launch autoware_cosim_http cosim.launch.py config_file:=/path/to/scenario.yaml
"""

import os
import subprocess
import time
import requests
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'config_file',
            description='Path to TeraSim scenario YAML file'
        ),
        DeclareLaunchArgument(
            'http_host',
            default_value='localhost',
            description='TeraSim HTTP API host'
        ),
        DeclareLaunchArgument(
            'http_port',
            default_value='8000',
            description='TeraSim HTTP API port'
        ),
        DeclareLaunchArgument(
            'map_path',
            default_value=os.path.expanduser('~/autoware/map'),
            description='Path to Autoware map directory'
        ),
        DeclareLaunchArgument(
            'control_cav',
            default_value='true',
            description='If true, Autoware controls CAV. If false, TeraSim controls CAV.'
        ),
        DeclareLaunchArgument(
            'terasim_path',
            default_value=os.path.expanduser('~/Projects/TeraSim'),
            description='Path to TeraSim directory'
        ),
        DeclareLaunchArgument(
            'perception_range',
            default_value='150.0',
            description='Only sync vehicles within this distance of CAV (meters). 0 = sync all.'
        ),

        # Use OpaqueFunction to start TeraSim and vehicle plugin
        OpaqueFunction(function=launch_cosim_nodes),
    ])


def launch_cosim_nodes(context):
    """Start TeraSim with run_experiments.py --autoware and launch vehicle plugin."""
    config_file = LaunchConfiguration('config_file').perform(context)
    http_host = LaunchConfiguration('http_host').perform(context)
    http_port = LaunchConfiguration('http_port').perform(context)
    map_path = LaunchConfiguration('map_path').perform(context)
    control_cav = LaunchConfiguration('control_cav').perform(context).lower() == 'true'
    terasim_path = LaunchConfiguration('terasim_path').perform(context)
    perception_range = float(LaunchConfiguration('perception_range').perform(context))

    base_url = f"http://{http_host}:{http_port}"

    # Start TeraSim with run_experiments.py --autoware
    # This runs SUMO at real-time (no tick driver needed)
    terasim_cmd = [
        'uv', 'run', 'python', 'scripts/run_experiments.py',
        '--config', config_file,
        '--autoware',
        '--map-path', map_path,
    ]
    print(f"[cosim] Starting TeraSim: {' '.join(terasim_cmd)}")

    terasim_process = ExecuteProcess(
        cmd=terasim_cmd,
        cwd=terasim_path,
        output='screen',
        name='terasim',
    )

    # Wait for TeraSim HTTP API to be ready
    print("[cosim] Waiting for TeraSim HTTP API...")
    simulation_id = None
    for i in range(60):
        try:
            resp = requests.get(f"{base_url}/simulations", timeout=2)
            if resp.status_code == 200:
                sims = resp.json()
                if sims:
                    simulation_id = list(sims.keys())[0]
                    print(f"[cosim] TeraSim ready, simulation_id: {simulation_id}")
                    break
        except:
            pass
        time.sleep(1)
        if i % 5 == 0:
            print(f"[cosim] Waiting for TeraSim... ({i}s)")

    if not simulation_id:
        print("[cosim] WARNING: Could not get simulation_id, using empty string")
        simulation_id = ""

    # Launch the vehicle plugin (no tick driver - SUMO ticks itself)
    vehicle_plugin = Node(
        package='autoware_cosim_http',
        executable='autoware_vehicle_plugin',
        name='autoware_vehicle_plugin',
        output='screen',
        parameters=[{
            'http_host': http_host,
            'http_port': int(http_port),
            'simulation_id': simulation_id,
            'map_path': map_path,
            'control_cav': control_cav,
            'perception_range': perception_range,
        }]
    )

    # Launch the TLS plugin for traffic light synchronization
    tls_plugin = Node(
        package='autoware_cosim_http',
        executable='autoware_tls_plugin',
        name='autoware_tls_plugin',
        output='screen',
        parameters=[{
            'http_host': http_host,
            'http_port': int(http_port),
            'simulation_id': simulation_id,
        }]
    )

    return [terasim_process, vehicle_plugin, tls_plugin]
