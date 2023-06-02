#include "ohrc_automation/cart_trajectory_impedance_controller.hpp"

bool CartTrajectoryImpedanceController::updateImpedanceTarget(const VectorXd& x, VectorXd& xd) {
  moveit_msgs::CartesianTrajectory trj;
  {
    std::lock_guard<std::mutex> lock(mtx_cart);

    if (_trj.points.empty())
      return false;
    trj = _trj;
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
