#include "ohrc_control/admittance_controller.hpp"
#include "ohrc_control/omega_controller.hpp"

class OmegaAdmittanceController : public AdmittanceController, public OmegaController {
  void initWithJnt(const KDL::JntArray& q_init) override;
  void getDesEffPoseVel(const double& dt, const KDL::JntArray& q_cur, const KDL::JntArray& dq_cur, KDL::Frame& des_eff_pose, KDL::Twist& des_eff_vel) override;

public:
  OmegaAdmittanceController();
};

OmegaAdmittanceController::OmegaAdmittanceController() {
}

void OmegaAdmittanceController::initWithJnt(const KDL::JntArray& q_init) {
  AdmittanceController::initWithJnt(q_init);
}

void OmegaAdmittanceController::getDesEffPoseVel(const double& dt, const KDL::JntArray& q_cur, const KDL::JntArray& dq_cur, KDL::Frame& des_eff_pose, KDL::Twist& des_eff_vel) {
  AdmittanceController::getDesEffPoseVel(dt, q_cur, dq_cur, des_eff_pose, des_eff_vel);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "marker_admittance_teleoperation");
  OmegaAdmittanceController OmegaAdmittanceController;
  if (OmegaAdmittanceController.control() < 0)
    ROS_ERROR("End by some fails");
}