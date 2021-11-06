cmake_minimum_required(VERSION 3.0.2)
project(ur_ros_custom)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/ur_node.py src/ur_custom_node.py src/vision_node.py src/ur_vision_node.py 
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})