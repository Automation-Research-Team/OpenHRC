
#include "ohrc_automation/cart_trajectory_impedance_controller.hpp"
#include "ohrc_control/single_interface.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "cart_trajectory_impedance_automation");
  SingleInterface<CartTrajectoryImpedanceController> controller;
  if (controller.control() < 0)
    ROS_ERROR("End by some fails");
}