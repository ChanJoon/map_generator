cmake_minimum_required(VERSION 3.5)
project(map_generator)

# Compiler settings
set(CMAKE_CXX_STANDARD 14)
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
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
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

install(TARGETS
  random_forest
  map_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
