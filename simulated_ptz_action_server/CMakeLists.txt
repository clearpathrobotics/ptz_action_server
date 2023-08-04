cmake_minimum_required(VERSION 3.0.2)
project(simulated_ptz_action_server)

find_package(catkin REQUIRED)

catkin_package(
)

include_directories(
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/ptz_camera_sim_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
