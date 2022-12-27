#include "ohrc_control/admittance_controller.hpp"

std::unique_ptr<AdmittanceController> admittanceController;

int main(int argc, char** argv) {
  ros::init(argc, argv, "admittance_teleoperation");
  admittanceController.reset(new AdmittanceController());
  if (admittanceController->control() < 0)
    ROS_ERROR("End by some fails");
}