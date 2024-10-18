cmake_minimum_required(VERSION 3.5)
project(ohrc_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


set(msg_files
  "msg/BodyPartState.msg"
  "msg/BodyState.msg"
  "msg/Contact.msg"
  "msg/Gripper.msg"
  "msg/HeadState.msg"
  "msg/State.msg"
  "msg/StateStamped.msg"
)
set(srv_files
  # "srv/CalcTwoFloats.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
)

# ament_export_dependencies(rosidl_default_runtime)
ament_package()