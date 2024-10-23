cmake_minimum_required(VERSION 3.0.2)
project(amc_base_msg)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  bms.msg
  car_speed.msg
  four_wheel_speed.msg
  four_wheel_odometer.msg
  four_wheel_loader.msg
  four_wheel_speed_can.msg
  four_wheel_odometer_can.msg
  four_wheel_loader_can.msg
)

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS  message_runtime std_msgs
)

