# Autoware

This is the Mcity-adapted version of Autoware Universe, based on the October 2023 release and extensively customized. For more information check out the official [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/main/)

## Installation
```bash
# Clone the repository.
git clone https://github.com/michigan-traffic-lab/autoware.git

# For the following instructions, we assume Autoware is cloned into your home directory.
cd autoware

# Install Autoware Dependencies using the provided Ansible script. 
# Note that this step involves changing some system settings. 
# When prompted to enter BECOME password, enter the system password.
# Restart your computer after this step.
./setup-dev-env.sh

# Install python depedencies
pip install redis

# Install c++ depedencies
sudo apt-get install libhiredis-dev libgeographic-dev libglm-dev

# Install ROS2 dependencies
rosdep init
rosdep update

# Open a new terminal to apply the changes.
cd autoware
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# Build packages (this can take 1-2 hours)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the autoware workspace
. install/setup.bash
```

## Launch Autoware
To launch the simulation:
```bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware/map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit lanelet2_map_file:=lanelet2_mcity_v43.osm
```

To run a quick startup demonstration, first initialize the ego vehicle's state by selecting **"2D Pose Estimate"** at the top of the RViz panel, then set the destination using **"2D Goal Pose"**. Finally, click **"Auto"** on the Operation Mode panel to start the vehicle. If Autoware consistently fails to plan a trajectory, See [Troubleshooting](#Troubleshooting) for a potential fix.

![tutorial](figure/startup_tutorial.gif)

To launch the real vehicle (this component is intended to run in a closed-loop setup with an actual Mcity vehicle; a CARLA co-simulation version will be released in the future):
```bash
ros2 launch autoware_launch autoware.launch.xml map_path:=$HOME/autoware/map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit lanelet2_map_file:=lanelet2_mcity_v43.osm
```

To Replay a rosbag:
```bash
ros2 launch autoware_launch logging_simulator.launch.xml map_path:=$HOME/autoware/map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit lanelet2_map_file:=lanelet2_mcity_v43.osm
```

## Post Installation Instructions (Optional)
To automatically source Autoware and avoid manual sourcing each time, follow these steps:

```bash
# Open your bash configuration file
sudo gedit ~/.bashrc

# Add the following line at the end of the file
source $HOME/autoware/install/setup.bash
```

## Troubleshooting
If Autoware fails to plan a trajectory after setting a **"2D Goal Pose"**, open a new terminal, run the following command, then relaunch Autoware and try again:

```bash
ros2 run autoware_cosim autoware_dummy_grid
```