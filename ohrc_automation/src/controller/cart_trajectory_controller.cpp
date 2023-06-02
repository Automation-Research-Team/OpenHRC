#include "ohrc_automation/cart_trajectory_controller.hpp"

void CartTrajectoryController::initInterface() {
  this->setSubscriber();
}

void CartTrajectoryController::setSubscriber() {
  trjSubscriber = n.subscribe<moveit_msgs::CartesianTrajectory>("/trj", 1000, &CartTrajectoryController::cbCartTrajectory, this, th);
}

void CartTrajectoryController::cbCartTrajectory(const moveit_msgs::CartesianTrajectory::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_cart);
  _trj = *msg;
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
  }

  static int i = 0;
  bool stop = false;
  if (i >= trj.points.size()) {
    stop = true;
  }

  Affine3d T_init = controller->getT_init();
  Affine3d target;
  tf2::fromMsg(trj.points[i].point.pose, target);
  target.translation() = T_init.translation() + target.translation();
  target.linear() = T_init.rotation();
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
