cmake_minimum_required(VERSION 3.8)
project(fk_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutoState.msg"
  "msg/Panel.msg"
  "msg/Layout.msg"
  "msg/PanelResult.msg"
  "msg/PanelResults.msg"
  "srv/AutoCommand.srv"
  DEPENDENCIES std_msgs
)

ament_package()
