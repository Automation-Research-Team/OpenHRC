
#include "ohrc_automation/cart_trajectory_minjerk_controller.hpp"
#include "ohrc_control/single_interface.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "cart_trajectory_automation");
  SingleInterface<CartTrajectoryMinjerkController> controller;
  if (controller.control() < 0)
    ROS_ERROR("End by some fails");
}