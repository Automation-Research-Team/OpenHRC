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
      initalPoint.point.pose = tf2::toMsg(T_init);

    trj.points.insert(trj.points.begin(), initalPoint);
  }

  _trj = this->interpolateTrajectory(trj);
  // std::cout << "subscribed" << std::endl;
}

void CartTrajectoryController::updateTargetPose(KDL::Frame& pos, KDL::Twist& twist) {
  moveit_msgs::CartesianTrajectory trj;
  {
    std::lock_guard<std::mutex> lock(mtx_cart);

    if (_trj.points.empty())
      return;
    trj = _trj;
  }

  static bool start = true;
  static ros::Time t_start;
  if (start) {
    t_start = ros::Time::now();
    start = false;
    i = 0;
  }

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

  if (stop) {
    controller->disableOperation();
    return;
  } else {
    controller->enableOperation();
  }
  //   std::cout << "i: " << i << std::endl;

  if ((ros::Time::now() - t_start).toNSec() > trj.points[i].time_from_start.toNSec())
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
