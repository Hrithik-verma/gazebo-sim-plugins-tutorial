cmake_minimum_required(VERSION 3.8)
project(yt_tutorial_gazebo_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
# may have to install 
# sudo apt install libgz-msgs10-dev
# sudo apt install libgz-transport13-dev
# apt install ros-jazzy-ros-gz-sim
find_package(ros_gz_bridge REQUIRED) 


install(
  DIRECTORY
    launch
    worlds
    configs
  DESTINATION
    share/${PROJECT_NAME}/
)




ament_package()