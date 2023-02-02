
#include "ohrc_automation/impedance_controller.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "impedance_automation");
  ImpedanceController controller;
  if (controller.control() < 0)
    ROS_ERROR("End by some fails");
}