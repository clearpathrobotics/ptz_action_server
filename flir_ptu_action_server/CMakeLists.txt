cmake_minimum_required(VERSION 3.0.2)
project(flir_ptu_action_server)

find_package(catkin REQUIRED COMPONENTS
  ptz_action_server_msgs
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS ptz_action_server_msgs sensor_msgs std_msgs)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/flir_ptu_action_server_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
