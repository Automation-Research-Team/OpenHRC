#include "ohrc_automation/dmp_controller.hpp"

void DmpController::initInterface() {
  ifstream file("../demos/cpp/json/Dmp_for_cpp.json");
  Dmp* dmp = json::parse(file).get<Dmp*>();
  //   VectorXd x(dmp->dim(), 1);
  //   VectorXd xd(dmp->dim(), 1);
  //   VectorXd y(dmp->dim_y(), 1);
  //   VectorXd yd(dmp->dim_y(), 1);
  //   VectorXd ydd(dmp->dim_y(), 1);

  //   dmp->integrateStart(x, xd);
}

void DmpController::updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) {
  //   dmp->integrateStep(dt, x, x, xd);
  //   // Convert complete DMP state to end-eff state
  //   dmp->stateAsPosVelAcc(x, xd, y, yd, ydd);
}
