cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(gz-cmake3 REQUIRED)

project(MovelModelTopicWayPlugin)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})



# MoveModelTopicWay
add_library(MoveModelTopicWay SHARED MoveModelTopicWay.cc )
set_property(TARGET MoveModelTopicWay PROPERTY CXX_STANDARD 17)
target_link_libraries(MoveModelTopicWay
  PUBLIC gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PUBLIC gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})



