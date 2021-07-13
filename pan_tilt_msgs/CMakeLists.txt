cmake_minimum_required(VERSION 2.8.3)
project(pan_tilt_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  message_runtime
  message_generation
)

add_message_files(
  FILES
  PanTiltCmdDeg.msg
  PanTiltCmdRad.msg
  PanTiltStatus.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pan_tilt_msgs
#  CATKIN_DEPENDS message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)