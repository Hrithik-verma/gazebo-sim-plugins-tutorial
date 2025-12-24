cmake_minimum_required(VERSION 3.8)
project(tutorial_gazebo_plugins)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# Gazebo (Gazebo Sim Harmonic) dependencies
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

## MoveModelTopicWay plugin
add_library(MoveModelTopicWay SHARED
  src/MoveModelTopicWay.cc
)

target_include_directories(MoveModelTopicWay
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(MoveModelTopicWay
  PUBLIC 
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
)



## MoveModelDirectRos plugin
add_library(MoveModelDirectRos SHARED
  src/MoveModelDirectRos.cc
)

target_include_directories(MoveModelDirectRos
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Non-ROS libraries: plain signature (no PUBLIC)
target_link_libraries(MoveModelDirectRos
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
)

# for ROS dependencies: use ament_target_dependencies
ament_target_dependencies(MoveModelDirectRos
  rclcpp
  std_msgs
)


## LightControl plugin
add_library(LightControl SHARED
  src/LightControl.cc
)

target_include_directories(LightControl
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(LightControl
  PUBLIC 
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
)


## LightControlServiceWay plugin
add_library(LightControlServiceWay SHARED
  src/LightControlServiceWay.cc
)

target_include_directories(LightControlServiceWay
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(LightControlServiceWay
  PUBLIC 
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
)


## LightControlDirectRos plugin
add_library(LightControlDirectRos SHARED
  src/LightControlDirectRos.cc
)

target_include_directories(LightControlDirectRos
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Non-ROS libraries: plain signature (no PUBLIC)
target_link_libraries(LightControlDirectRos
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
)

# for ROS dependencies: use ament_target_dependencies
ament_target_dependencies(LightControlDirectRos
  rclcpp
  std_srvs
)


## JointControl plugin
add_library(JointControl SHARED
  src/JointControl.cc
)

target_include_directories(JointControl
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(JointControl
  PUBLIC 
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
)


install(
  TARGETS 
  MoveModelTopicWay
  MoveModelDirectRos
  LightControl
  LightControlServiceWay
  LightControlDirectRos
  JointControl
  LIBRARY DESTINATION share/${PROJECT_NAME}/plugins
)



# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)



ament_package()
