#include "ohrc_control/multi_test_controller.hpp"

MultiTestController::MultiTestController() {
  // axis = 1;
  // A = 0.25;
  // f = 0.1;
}

void MultiTestController::runLoopEnd() {
  // TODO: Save solusion accuracy and calcuration time
}

void MultiTestController::updateManualTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) {
  static ros::Time t0 = ros::Time::now();
  tf::transformEigenToKDL(controller->getT_init(), pose);
  int axis = 1;
  double A = 0.15;
  double f = 0.2;
  double t = (ros::Time::now() - t0).toSec();
  pose.p.data[axis] = A * sin(2. * M_PI * f * t) + pose.p.data[axis];
  twist.vel.data[axis] = A * 2. * M_PI * f * cos(2. * M_PI * f * t);

  axis = 2;
  A = 0.15;
  f = 0.1;
  pose.p.data[axis] = -A * sin(2. * M_PI * f * t) + pose.p.data[axis];
  twist.vel.data[axis] = -A * 2. * M_PI * f * cos(2. * M_PI * f * t);
  controller->enableOperation();
}

void MultiTestController::updateAutoTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) {
  static bool prev_highPriority = true;
  bool highPriority;
  if (controller->getT_root().translation()(2) < pose.p.data[2]) {
    multimyik_solver_ptr->setRobotWeight(controller->getIndex(), 100.0);
    highPriority = true;
  } else {
    multimyik_solver_ptr->setRobotWeight(controller->getIndex(), 0.01);
    highPriority = false;
  }
  if (highPriority != prev_highPriority) {
    if (highPriority)
      ROS_INFO_STREAM("Automation priority is switched to HIGH");
    else
      ROS_INFO_STREAM("Automation priority is switched to LOW");
  }
  prev_highPriority = highPriority;

  static ros::Time t0 = ros::Time::now();
  tf::transformEigenToKDL(controller->getT_init(), pose);
  int axis = 2;
  double A = 0.15;
  double f = 0.1;
  double t = (ros::Time::now() - t0).toSec();
  pose.p.data[axis] = -A * sin(2. * M_PI * f * t) + pose.p.data[axis];
  twist.vel.data[axis] = -A * 2. * M_PI * f * cos(2. * M_PI * f * t);
  controller->enableOperation();
}