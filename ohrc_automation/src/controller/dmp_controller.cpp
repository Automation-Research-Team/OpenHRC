#include "ohrc_automation/dmp_controller.hpp"

void DmpController::initInterface() {
  std::string file_name;
  n.getParam("dmp_config_json", file_name);
  ifstream file(file_name);
  dmp = json::parse(file).get<Dmp*>();
  if (dmp == nullptr) {
    ROS_ERROR("Failed to load DMP");
    return;
  }

  //   dmp->integrateStart(x, xd);
}

void DmpController::updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) {
  controller->startOperation();
  VectorXd x(dmp->dim(), 1);
  VectorXd xd(dmp->dim(), 1);
  VectorXd y(dmp->dim_y(), 1);
  VectorXd yd(dmp->dim_y(), 1);
  VectorXd ydd(dmp->dim_y(), 1);

  x << pose.p[0], pose.p[1], pose.p[2], twist.vel[0], twist.vel[1], twist.vel[2];

  // KDL::Frame pose;
  // KDL::Twist twist;
  // controller->getCartState(pose, twist);

  // x << pose.p.x(), pose.p.y(), pose.p.z();
  if (firstLoop) {
    dmp->integrateStart(x, xd);

    firstLoop = false;
  }

  dmp->integrateStep(dt, x, x, xd);

  // Convert complete DMP state to end-eff state
  dmp->stateAsPosVelAcc(x, xd, y, yd, ydd);

  pose.p[0] = y(0);
  pose.p[1] = y(1);
  pose.p[2] = y(2);

  twist.vel[0] = yd(0);
  twist.vel[1] = yd(1);
  twist.vel[2] = yd(2);
}
