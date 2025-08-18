cmake_minimum_required(VERSION 3.8)
project(ndt_localizer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msgs/NdtStat.msg"
  DEPENDENCIES std_msgs
)

add_executable(voxel_grid_filter nodes/voxel_grid_filter.cpp)
ament_target_dependencies(voxel_grid_filter rclcpp sensor_msgs std_msgs PCL pcl_conversions )

target_link_libraries(voxel_grid_filter ${PCL_LIBRARIES})

add_executable(map_loader nodes/mapLoader.cpp)
ament_target_dependencies(map_loader rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs PCL pcl_conversions)
target_link_libraries(map_loader ${PCL_LIBRARIES})

add_executable(ndt_localizer_node nodes/ndt.cpp)
ament_target_dependencies(ndt_localizer_node rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs diagnostic_msgs PCL tf2 tf2_ros tf2_geometry_msgs tf2_eigen pcl_conversions)
target_link_libraries(ndt_localizer_node ${PCL_LIBRARIES})

install(TARGETS voxel_grid_filter map_loader ndt_localizer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py" PATTERN "*.launch"
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.urdf" PATTERN "*.dae" PATTERN "*.jpg"
)
install(DIRECTORY cfgs/
  DESTINATION share/${PROJECT_NAME}/cfgs
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY map/
  DESTINATION share/${PROJECT_NAME}/map
)

ament_package()