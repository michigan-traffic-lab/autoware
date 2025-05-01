# Autoware

This is the Mcity-adapted version of Autoware Universe, based on the October 2023 release and extensively customized. For more information check out the official [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/main/)

## Installation
```bash
# Clone the repository
git clone https://github.com/QZJGeorge/autoware.git

cd autoware

# Install Autoware Dependencies using the provided Ansible script. 
# Note that this step involves chaging some system settings. 
# When prompted to enter BECOME password, enter system password,
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

# Build packages
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the autoware workspace
. install/setup.bash
```

## Launch Terasim Autoware
To launch the simulation
```bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware/map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit lanelet2_map_file:=lanelet2_mcity_v43.osm
```

To launch the real car
```bash
ros2 launch autoware_launch autoware.launch.xml map_path:=$HOME/autoware/map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit lanelet2_map_file:=lanelet2_mcity_v43.osm
```

To Replay a rosbag
```bash
ros2 launch autoware_launch logging_simulator.launch.xml map_path:=$HOME/autoware/map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit lanelet2_map_file:=lanelet2_mcity_v43.osm
```
