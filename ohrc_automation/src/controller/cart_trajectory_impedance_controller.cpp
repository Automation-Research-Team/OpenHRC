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
  static moveit_msgs::CartesianTrajectory trj;
  static bool updateTrjs = true;
  {
    std::lock_guard<std::mutex> lock(mtx_cart);

    if (!_flagTrjs || _trjs.size() == 0)
      return false;

    if (updateTrjs) {
      nTrjs = _trjs.size();
      trj = _trjs[targetIdx];
    }
    updateTrjs = false;
  }
  int nTrj = trj.points.size();

  static ros::Time t_start = ros::Time::now();
  static int t_i = 0;
  static int forward = 1;

  // substitute the target pose at the current time step t_i
  Vector3d pose, vel;
  tf2::fromMsg(trj.points[t_i].point.pose.position, pose);
  tf2::fromMsg(trj.points[t_i].point.velocity.linear, vel);
  xd << pose + this->restPose.translation(), vel * forward;

  // increment t_i in [0, nTrj - 1]
  bool a = (forward - 1) / (-2);  // 0 or 1
  double t_ns = (trj.points[nTrj - 1].time_from_start.toNSec() * (double)a + std::pow((-1.0), a) * trj.points[t_i].time_from_start.toNSec());
  if ((ros::Time::now() - t_start).toNSec() > t_ns)
    t_i = std::max(std::min(t_i + forward, (int)(trj.points.size() - 1)), 0);

  // check if the target pose is reached
  Vector3d x_target_end;
  if (forward == 1)
    tf2::fromMsg(trj.points[nTrj - 1].point.pose.position, x_target_end);  // might be better to use restPose
  else
    tf2::fromMsg(trj.points[0].point.pose.position, x_target_end);         // might be better to use targetPose
  VectorXd x_target = (VectorXd(6) << this->restPose.translation() + x_target_end, VectorXd::Zero(3)).finished();
  TaskState taskState = this->updataTaskState(x - x_target, forward);

  // if reaching was succeeded or failed, update the target pose (forward = 1) or go back to the initial pose (forward = -1)
  if (taskState == TaskState::Success || taskState == TaskState::Fail) {
    forward *= -1;
    t_start = ros::Time::now();
  }

  // if reaching was succeeded, move on to the next target pose
  if (taskState == TaskState::Success && forward == 1) {
    updateTrjs = true;
    targetIdx++;

    if (targetIdx > nTrjs - 1)
      targetIdx = 0;
  }

  return true;
}
