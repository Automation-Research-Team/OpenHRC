#include "ohrc_control/omega_controller.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "omega_teleoperation");
  OmegaController OmegaController;
  if (OmegaController.control() < 0)
    ROS_ERROR("End by some fails");
}