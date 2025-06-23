cmake_minimum_required(VERSION 3.5)
project(ndt_localizer)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msgs/NdtStat.msg"
  DEPENDENCIES std_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(voxel_grid_filter nodes/voxel_grid_filter.cpp)
ament_target_dependencies(voxel_grid_filter rclcpp sensor_msgs pcl_conversions pcl_msgs tf2_eigen)

add_executable(mapLoader nodes/mapLoader.cpp)
ament_target_dependencies(mapLoader rclcpp sensor_msgs geometry_msgs nav_msgs diagnostic_msgs pcl_conversions pcl_msgs tf2 tf2_ros tf2_geometry_msgs tf2_eigen)

add_executable(ndt nodes/ndt.cpp)
ament_target_dependencies(ndt rclcpp sensor_msgs geometry_msgs nav_msgs diagnostic_msgs pcl_conversions pcl_msgs tf2 tf2_ros tf2_geometry_msgs tf2_eigen)

install(TARGETS
  voxel_grid_filter
  mapLoader
  ndt
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()