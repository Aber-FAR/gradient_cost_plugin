cmake_minimum_required(VERSION 3.5)
project(gradient_cost_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(lib_name gradient_cost_plugin_core)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2
    tf2_ros
    nav_msgs
    sensor_msgs
    geometry_msgs
    pluginlib
    costmap_2d
    grid_map_ros
    grid_map_core
    grid_map_costmap_2d
    message_filters
    dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
    cfg/GradientCostPlugin.cfg
)

catkin_package(
    CATKIN_DEPENDS roscpp pluginlib costmap_2d grid_map_ros grid_map_costmap_2d message_filters dynamic_reconfigure
)

add_library(${lib_name} SHARED
    src/gradient_cost_layer.cpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

install(FILES gradient_cost_layer.xml
    TARGETS ${lib_name}
    DESTINATION lib
)

