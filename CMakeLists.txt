cmake_minimum_required(VERSION 3.5)
project(map_generator)

# Compiler settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${SDL_INCLUDE_DIRS}
  ${SDL_IMAGE_INCLUDE_DIRS}
)

add_executable(random_forest src/random_forest_sensing.cpp)
ament_target_dependencies(random_forest
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
)

target_link_libraries(random_forest ${PCL_LIBRARIES})

add_executable(map_pub src/map_publisher.cpp)
ament_target_dependencies(map_pub
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
)

target_link_libraries(map_pub ${PCL_LIBRARIES})

add_executable(structure_map src/structure_map.cpp)
ament_target_dependencies(structure_map
  rclcpp
  std_msgs
  geometry_msgs
  pcl_conversions
  nav_msgs
  visualization_msgs
  rosbag2_cpp
)
target_link_libraries(structure_map ${PCL_LIBRARIES})

add_executable(read_grid_map src/read_grid_map.cpp)
ament_target_dependencies(read_grid_map
  rclcpp
  std_msgs
  geometry_msgs
  pcl_conversions
  nav_msgs
  visualization_msgs
  rosbag2_cpp
)
target_link_libraries(read_grid_map
  ${PCL_LIBRARIES}
  ${SDL_LIBRARY}
  ${SDL_IMAGE_LIBRARIES}
)

add_executable(mockamap_node
  src/mockamap.cpp
  src/maps.cpp
  src/perlinnoise.cpp
)
ament_target_dependencies(mockamap_node
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(mockamap_node ${PCL_LIBRARIES})

install(TARGETS
  random_forest
  map_pub
  structure_map
  read_grid_map
  mockamap_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
