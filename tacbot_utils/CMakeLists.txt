cmake_minimum_required(VERSION 2.8.3)
project(tacbot_utils)

find_package(catkin REQUIRED COMPONENTS rospy)
catkin_package()


# Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/gazebo_link_pose_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
