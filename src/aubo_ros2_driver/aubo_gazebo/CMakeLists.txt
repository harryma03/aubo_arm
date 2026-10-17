cmake_minimum_required(VERSION 3.5)
project(aubo_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(jaka_initial_pose_plugin SHARED
  src/jaka_initial_pose_plugin.cpp
)

target_link_libraries(jaka_initial_pose_plugin
  ${GAZEBO_LIBRARIES}
)

install(DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS jaka_initial_pose_plugin
  LIBRARY DESTINATION lib
)

ament_package()
