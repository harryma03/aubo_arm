cmake_minimum_required(VERSION 2.8.3)
project(aubo_description)

if (DEFINED ENV{ROS_DISTRO})
    find_package(ros_environment REQUIRED)
    set(ROS_VERSION $ENV{ROS_VERSION})
    message("ROS_VERSION" ${ROS_VERSION})
    if(${ROS_VERSION} EQUAL 1)
        
    	execute_process(COMMAND bash ros1_xacro.sh
    	                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
    	find_package(catkin REQUIRED)

    	catkin_package()

    	install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
    	install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
    	install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
    endif()
    if(${ROS_VERSION} EQUAL 2)
        cmake_minimum_required(VERSION 3.8)
        find_package(ament_cmake REQUIRED)
        install(DIRECTORY launch meshes urdf rviz DESTINATION share/${PROJECT_NAME})
        ament_package()
    endif()
    endif()
