# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.8)
project(lrs_assignment1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# --------------------------------------------------
# Voxel map
# --------------------------------------------------

add_library(voxel_map
  src/voxel_map.cpp
)

target_link_libraries(voxel_map
  ${PCL_LIBRARIES}
)

# --------------------------------------------------
# A* planner
# --------------------------------------------------

add_library(astar_planner
  src/astar_planner.cpp
)

target_link_libraries(astar_planner
  voxel_map
)

# --------------------------------------------------
# Map loader test
# --------------------------------------------------

add_executable(map_loader
  src/map_loader.cpp
)

target_link_libraries(map_loader
  voxel_map
  ${PCL_LIBRARIES}
)

# --------------------------------------------------
# Planner test
# --------------------------------------------------

add_executable(planner_test
  src/planner_test.cpp
)

target_link_libraries(planner_test
  astar_planner
  voxel_map
  ${PCL_LIBRARIES}
)

# --------------------------------------------------

install(
  TARGETS
    voxel_map
    astar_planner
    map_loader
    planner_test
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

add_executable(mission_node src/mission_node.cpp)
target_link_libraries(mission_node astar_planner voxel_map)
ament_target_dependencies(mission_node rclcpp geometry_msgs sensor_msgs std_msgs std_srvs mavros_msgs nav_msgs)
install(TARGETS mission_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config missions DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/analyze_bag.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  add_executable(core_tests test/core_tests.cpp)
  add_test(NAME core_tests COMMAND core_tests)
  add_executable(path_guard_tests test/path_guard_tests.cpp)
  target_link_libraries(path_guard_tests astar_planner voxel_map)
  add_test(NAME path_guard_tests COMMAND path_guard_tests)
  add_executable(flight_clearance test/flight_clearance.cpp)
  target_link_libraries(flight_clearance astar_planner voxel_map)
endif()
ament_package()