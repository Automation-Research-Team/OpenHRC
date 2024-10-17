cmake_minimum_required(VERSION 3.5)
project(ohrc_control)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


find_package(OsqpEigen REQUIRED OsqpEigen)
find_package(magic_enum CONFIG REQUIRED magic_enum)


ament_auto_add_library(${PROJECT_NAME} 
  src/my_ik.cpp
  src/cart_controller.cpp
  src/multi_cart_controller.cpp
  src/position_feedback_controller.cpp
  src/hybrid_feedback_controller.cpp
)
target_link_libraries(${PROJECT_NAME} OsqpEigen::OsqpEigen magic_enum::magic_enum)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_auto_package()