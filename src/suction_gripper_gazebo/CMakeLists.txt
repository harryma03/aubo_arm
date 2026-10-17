cmake_minimum_required(VERSION 3.8)
project(suction_gripper_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(suction_attach_plugin SHARED
  src/suction_attach_plugin.cpp
)

ament_target_dependencies(
  suction_attach_plugin
  rclcpp
  std_srvs
  gazebo_ros
)

target_link_libraries(
  suction_attach_plugin
  ${GAZEBO_LIBRARIES}
)

install(
  TARGETS suction_attach_plugin
  LIBRARY DESTINATION lib
)

ament_package()
