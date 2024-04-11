cmake_minimum_required(VERSION 3.5)
project(ptz_action_server_msgs)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        action/PtzMove.action
        action/PtzFrame.action
        action/PtzRelTo.action
        msg/PtzState.msg
)

ament_package()
