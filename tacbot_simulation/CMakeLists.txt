cmake_minimum_required(VERSION 2.8.11)
project(tacbot_simulation)

set(CMAKE_CXX_FLAGS "-std=c++1y ${CMAKE_CXX_FLAGS} ")
#set(CMAKE_BUILD_TYPE Release)

#catkin_add_env_hooks(tacbot_sim SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

find_package(gazebo REQUIRED)

find_package ( catkin REQUIRED COMPONENTS
    roscpp
    std_msgs sensor_msgs geometry_msgs nav_msgs
    tf tf2_ros gazebo_ros gazebo_plugins
    )

catkin_package (
    INCLUDE_DIRS src
    LIBRARIES DriveControllerNode
    CATKIN_DEPENDS
    roscpp
    std_msgs sensor_msgs geometry_msgs nav_msgs
    tf tf2_ros gazebo_ros gazebo_plugins
    )

include_directories ( 
    src
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    )

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(gazebo_ros_tacbot src/gazebo_ros_tacbot.cpp)
target_link_libraries(gazebo_ros_tacbot ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_tacbot_imu src/gazebo_tacbot_imu.cpp)
target_link_libraries(gazebo_tacbot_imu ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


install(TARGETS gazebo_ros_tacbot
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY maps
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
