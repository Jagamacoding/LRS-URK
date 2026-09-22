cmake_minimum_required(VERSION 3.8)
project(lrs_assignment1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
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

ament_package()