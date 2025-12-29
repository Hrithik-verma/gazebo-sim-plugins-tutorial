cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(gz-cmake3 REQUIRED)

project(MoveModelTopicWayPimpPimpPlugin)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})



# MoveModelTopicWayPimp
add_library(MoveModelTopicWayPimp SHARED MoveModelTopicWayPimp.cc )
set_property(TARGET MoveModelTopicWayPimp PROPERTY CXX_STANDARD 17)
target_link_libraries(MoveModelTopicWayPimp
  PUBLIC gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PUBLIC gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})



