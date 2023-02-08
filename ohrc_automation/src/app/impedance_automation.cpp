
#include "ohrc_automation/impedance_controller.hpp"
#include "ohrc_control/single_control.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "impedance_automation");
  SingleControl<ImpedanceController> controller;
  if (controller.control() < 0)
    ROS_ERROR("End by some fails");
}