#include "ohrc_control/multi_omega_controller.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "omega_teleoperation");
  MultiOmegaController MultiOmegaController;
  if (MultiOmegaController.control() < 0)
    ROS_ERROR("End by some fails");
}