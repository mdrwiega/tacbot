cmake_minimum_required(VERSION 2.8.11)
project(tacbot_description)

find_package(catkin REQUIRED COMPONENTS urdf xacro tacbot_simulation gazebo_plugins)

catkin_package(
  CATKIN_DEPENDS urdf xacro tacbot_simulation gazebo_plugins
)

install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
