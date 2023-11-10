cmake_minimum_required(VERSION 3.0.2)
project(ptz_action_server_msgs)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  actionlib_msgs
  std_msgs
)

add_action_files(
  FILES
  Ptz.action
  PtzFrame.action
  PtzRelTo.action
)

add_message_files(
  FILES
  PtzState.msg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS actionlib_msgs genmsg std_msgs)
