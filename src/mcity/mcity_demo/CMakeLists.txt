cmake_minimum_required(VERSION 3.14)
project(mcity_demo)

find_package(autoware_cmake REQUIRED)

autoware_package()

# Add include directories
include_directories(
    include
    ../redis_client/
    /usr/include/hiredis
)

########################## AUTOWARE INTERFACE COSIM ####################################

ament_auto_add_library(autoware_interface_demo_cosim_component SHARED
  src/autoware_interface_demo_cosim.cpp
  ../redis_client/RedisClient.cpp
)

target_link_libraries(autoware_interface_demo_cosim_component hiredis)

rclcpp_components_register_node(autoware_interface_demo_cosim_component
  PLUGIN "autoware_interface_demo_cosim::AutowareInterfaceDemoCosim"
  EXECUTABLE autoware_interface_demo_cosim
)

########################### AUTOWARE INTERFACE REALCAR ###################################

ament_auto_add_library(autoware_interface_demo_realcar_component SHARED
  src/autoware_interface_demo_realcar.cpp
)

rclcpp_components_register_node(autoware_interface_demo_realcar_component
  PLUGIN "autoware_interface_demo_realcar::AutowareInterfaceDemoRealcar"
  EXECUTABLE autoware_interface_demo_realcar
)

########################################################################################

ament_auto_package(
  INSTALL_TO_SHARE
  launch  
  include
)
