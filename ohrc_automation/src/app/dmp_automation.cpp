
#include "ohrc_automation/dmp_controller.hpp"
#include "ohrc_control/single_interface.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "dmp_automation");
  SingleInterface<DmpController> controller;
  if (controller.control() < 0)
    ROS_ERROR("End by some fails");
}