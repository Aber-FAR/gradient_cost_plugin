cmake_minimum_required(VERSION 3.8)
project(gradient_cost_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(lib_name gradient_cost_plugin_core)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_costmap_2d REQUIRED)

set(dep_pkgs
    rclcpp
    pluginlib
    nav2_costmap_2d
    grid_map_ros
    grid_map_costmap_2d
    )

add_library(${lib_name} SHARED
            src/gradient_cost_layer.cpp
            )

include_directories(include
                    )

install(TARGETS ${lib_name}
        DESTINATION lib)

pluginlib_export_plugin_description_file(nav2_costmap_2d gradient_cost_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
