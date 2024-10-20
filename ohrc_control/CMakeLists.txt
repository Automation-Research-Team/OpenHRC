cmake_minimum_required(VERSION 3.5)
project(ohrc_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(std_msgs REQUIRED)


find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)
find_package(ohrc_common REQUIRED)
find_package(ohrc_msgs REQUIRED)


find_package(OsqpEigen REQUIRED)
find_package(magic_enum CONFIG REQUIRED)

add_library(${PROJECT_NAME}  SHARED src/my_ik.cpp
                              # src/multi_my_ik.cpp 
                              src/cart_controller.cpp 
                              src/multi_cart_controller.cpp 
                              # src/forwarding_controller.cpp
                              src/admittance_controller.cpp
                              src/position_feedback_controller.cpp
                              src/hybrid_feedback_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )


ament_target_dependencies(${PROJECT_NAME} rclcpp trac_ik_lib interactive_markers control_msgs tf2_kdl tf2_eigen tf2_eigen_kdl std_msgs std_srvs ohrc_common ohrc_msgs urdf)

target_link_libraries(${PROJECT_NAME} OsqpEigen::OsqpEigen magic_enum::magic_enum)


# ament_export_dependencies(rclcpp trac_ik_lib interactive_markers control_msgs tf2_kdl tf2_eigen tf2_eigen_kdl std_msgs std_srvs ohrc_common ohrc_msgs urdf)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
 config
 DESTINATION share/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()
# ament_auto_package()
ament_package()