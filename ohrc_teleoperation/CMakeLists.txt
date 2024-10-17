cmake_minimum_required(VERSION 3.5)
project(ohrc_teleoperation)


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OsqpEigen REQUIRED)
find_package(magic_enum CONFIG REQUIRED)

set(interfaces
  marker
  # state_topic
  # twist_topic
  # pose_topic
)

foreach(interface ${interfaces})
  ament_auto_add_executable(${interface}_teleoperation src/interface/${interface}_interface.cpp src/app/${interface}_teleoperation.cpp)
  target_link_libraries(${interface}_teleoperation OsqpEigen::OsqpEigen magic_enum::magic_enum )
endforeach(interface)

# set(state_topic_interfaces
#   omega
#   xr_body
# )

# foreach(interface ${state_topic_interfaces})
#   add_executable(${interface}_teleoperation src/interface/${interface}_interface.cpp src/interface/state_topic_interface.cpp src/app/${interface}_teleoperation.cpp)
#   add_dependencies(${interface}_teleoperation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#   target_link_libraries(${interface}_teleoperation ${catkin_LIBRARIES} OsqpEigen::OsqpEigen magic_enum::magic_enum)
# endforeach(interface)

# set(body_interfaces
#   k4abt_body
# )

# foreach(interface ${body_interfaces})
#   add_executable(${interface}_teleoperation src/interface/${interface}_interface.cpp src/interface/state_topic_interface.cpp src/interface/xr_body_interface.cpp src/app/${interface}_teleoperation.cpp)
#   add_dependencies(${interface}_teleoperation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#   target_link_libraries(${interface}_teleoperation ${catkin_LIBRARIES} OsqpEigen::OsqpEigen magic_enum::magic_enum)
# endforeach(interface)

# set(twist_topic_interfaces
#   joy_topic
# )

# foreach(interface ${twist_topic_interfaces})
#   add_executable(${interface}_teleoperation src/interface/${interface}_interface.cpp src/interface/twist_topic_interface.cpp src/app/${interface}_teleoperation.cpp)
#   add_dependencies(${interface}_teleoperation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#   target_link_libraries(${interface}_teleoperation ${catkin_LIBRARIES} OsqpEigen::OsqpEigen magic_enum::magic_enum)
# endforeach(interface)

# set(w_twist_topic_interfaces
#   marker
# )

# foreach(interface ${w_twist_topic_interfaces})
#   add_executable(${interface}_twist_topic_teleoperation src/interface/${interface}_interface.cpp src/interface/twist_topic_interface.cpp src/app/${interface}_twist_topic_teleoperation.cpp)
#   add_dependencies(${interface}_twist_topic_teleoperation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#   target_link_libraries(${interface}_twist_topic_teleoperation ${catkin_LIBRARIES} OsqpEigen::OsqpEigen magic_enum::magic_enum)
# endforeach(interface)

ament_auto_package()