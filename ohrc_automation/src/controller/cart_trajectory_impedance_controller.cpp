#include "ohrc_automation/cart_trajectory_impedance_controller.hpp"

void CartTrajectoryImpedanceController::initInterface() {
  ImpedanceController::initInterface();
  this->client = n.serviceClient<ohrc_msgs::GetTrajectories>("/trajectory_generation_service");
}

void CartTrajectoryImpedanceController::cbTargetPoses(const geometry_msgs::PoseArray::ConstPtr& msg) {
  std::vector<moveit_msgs::CartesianTrajectory> trjs;
  if (!this->getTrajectories(*msg, trjs))
    return;

  std::lock_guard<std::mutex> lock(mtx_cart);
  this->_trjs = trjs;
  _flagTrjs = true;
}

bool CartTrajectoryImpedanceController::getTrajectories(const geometry_msgs::PoseArray& targetPoses, std::vector<moveit_msgs::CartesianTrajectory>& trjs) {
  std::vector<Affine3d> targets;
  tf2::fromMsg(targetPoses, targets);

  ohrc_msgs::GetTrajectories srv;
  srv.request.targetPoses.poses.resize(targets.size());
  for (int i = 0; i < targets.size(); i++) {
    targets[i].translation() = targets[i].translation() - this->restPose.translation();
    srv.request.targetPoses.poses[i] = tf2::toMsg(targets[i]);
  }

  if (this->client.call(srv)) {
    trjs = srv.response.trjs;
    return true;
  } else {
    ROS_ERROR("Failed to call trajectory generation service");
    return false;
  }
  ROS_WARN_STREAM(__LINE__);
}

bool CartTrajectoryImpedanceController::updateImpedanceTarget(const VectorXd& x, VectorXd& xd) {
  moveit_msgs::CartesianTrajectory trj;
  {
    std::lock_guard<std::mutex> lock(mtx_cart);

    if (!_flagTrjs)
      return false;
    trj = _trjs[0];
  }

  static bool start = true;
  static ros::Time t_start;

  int nTrj = trj.points.size();
  if (start) {
    t_start = ros::Time::now();
    start = false;
  }

  static int i = 0;
  static int forward = 1;

  Affine3d target;
  tf2::fromMsg(trj.points[i].point.pose, target);
  target.translation() = this->restPose.translation() + target.translation();
  target.linear() = this->restPose.rotation();

  xd << target.translation(), trj.points[i].point.velocity.linear.x, trj.points[i].point.velocity.linear.y, trj.points[i].point.velocity.linear.z;
  xd.tail(3) *= forward;

  bool a = (forward - 1) / (-2);  // 0 or 1
  if ((ros::Time::now() - t_start).toNSec() > (trj.points[nTrj - 1].time_from_start.toNSec() * (double)a + std::pow((-1.0), a) * trj.points[i].time_from_start.toNSec()))
    i = i + 1 * forward;

  if (i > trj.points.size() - 1 || i < 0) {
    i = i - forward;
    TaskState taskState = this->updataTaskState(x - xd, forward);

    if (taskState == TaskState::Success || taskState == TaskState::Fail) {
      forward *= -1;
      start = true;
    }
  }

  return true;
}
