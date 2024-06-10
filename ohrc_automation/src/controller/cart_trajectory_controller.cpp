#include "ohrc_automation/cart_trajectory_controller.hpp"

void CartTrajectoryController::initInterface() {
  this->setSubscriber();
  T_init = controller->getT_cur();
  n.getParam("relative", relative);

  // controller->enablePoseFeedback();
}

void CartTrajectoryController::setSubscriber() {
  trjSubscriber = n.subscribe<moveit_msgs::CartesianTrajectory>("/trj", 1, &CartTrajectoryController::cbCartTrajectory, this, th);
}

void CartTrajectoryController::cbCartTrajectory(const moveit_msgs::CartesianTrajectory::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_cart);
  moveit_msgs::CartesianTrajectory trj = *msg;

  if (trj.points[0].time_from_start.toSec() > dt) {  // trj does not include initial state
    moveit_msgs::CartesianTrajectoryPoint initalPoint;
    if (!relative)
      initalPoint.point.pose = tf2::toMsg(controller->getT_cur());

    trj.points.insert(trj.points.begin(), initalPoint);
  }

  this->modifyTrajectory(trj);

  _trj = this->interpolateTrajectory(trj);
  _newTrj = true;
  // std::cout << "subscribed" << std::endl;
}

void CartTrajectoryController::updateTargetPose(KDL::Frame& pos, KDL::Twist& twist) {
  moveit_msgs::CartesianTrajectory trj;
  bool newTrj = false;
  {
    std::lock_guard<std::mutex> lock(mtx_cart);

    if (_trj.points.empty())
      return;
    trj = _trj;
    newTrj = _newTrj;
    _newTrj = false;
  }

  static bool start = true;
  static ros::Time t_start;

  if (newTrj)
    start = true;

  if (start) {
    t_start = ros::Time::now();
    start = false;
    T_init = controller->getT_cur();
    i = 0;
  }

  double t = (ros::Time::now() - t_start).toSec();

  bool stop = false;
  if (i >= trj.points.size()) {
    i = trj.points.size() - 1;
    stop = true;
  }

  Affine3d target;
  tf2::fromMsg(trj.points[i].point.pose, target);

  if (relative) {
    target.translation() = T_init.translation() + target.translation();
    target.linear() = T_init.rotation() * target.rotation();
  }
  //   target.rotation() = T_init.rotation()

  tf::transformEigenToKDL(target, pos);
  tf2::fromMsg(trj.points[i].point.velocity, twist);

  Affine3d finalTarget;
  tf2::fromMsg(trj.points[trj.points.size() - 1].point.pose, finalTarget);
  if (relative) {
    finalTarget.translation() = T_init.translation() + finalTarget.translation();
    finalTarget.linear() = T_init.rotation() * finalTarget.rotation();
  }
  this->e = MyIK::getCartError(controller->getT_cur(), finalTarget);
  // std::cout << e.transpose() << std::endl;

  if (stop) {
    controller->disableOperation();
    return;
  } else {
    controller->enableOperation();
  }
  //   std::cout << "i: " << i << std::endl;

  if (t > trj.points[i].time_from_start.toSec())
    i++;
}

void CartTrajectoryController::resetInterface() {
  T_init = controller->getT_cur();
  start = true;
  _trj = moveit_msgs::CartesianTrajectory();
  _trj.points.resize(1);
  _trj.points[0].time_from_start = ros::Duration(1.0);
  // controller->enablePoseFeedback();
}
