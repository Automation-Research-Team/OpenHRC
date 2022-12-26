#include "ohrc_control/admittance_controller.hpp"
#include "ohrc_control/marker_controller.hpp"

class MarkerAdmittanceController : public AdmittanceController, public MarkerController {
  void initWithJnt(const KDL::JntArray& q_init) override;
};

void MarkerAdmittanceController::initWithJnt(const KDL::JntArray& q_init) {
  resetFt();
  configMarker(q_init);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "marker_admittance_teleoperation");
  MarkerAdmittanceController MarkerAdmittanceController;
  if (MarkerAdmittanceController.control() < 0)
    ROS_ERROR("End by some fails");
}