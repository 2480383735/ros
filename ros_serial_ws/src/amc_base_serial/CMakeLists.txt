cmake_minimum_required(VERSION 3.0.2)
project(amc_base_serial)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  rospy
  tf
  serial
  amc_base_msg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hello_world
 CATKIN_DEPENDS roscpp rospy std_msgs tf  serial
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(amc_ros_cpp_node src/amc_ros_cpp_node.cpp)
add_dependencies(amc_ros_cpp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(amc_ros_cpp_node
  ${catkin_LIBRARIES}
)


catkin_install_python(PROGRAMS
  scripts/amc_base_node_serial.py
  scripts/amc_base_cmd_vel_publish.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
