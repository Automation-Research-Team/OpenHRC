#include "ohrc_automation/impedance_controller.hpp"

void ImpedanceController::initInterface() {
  this->impParam = getImpParam(getImpCoeff());

  this->getParam();

  RespawnReqPublisher = n.advertise<std_msgs::Empty>(this->targetName + "/success", 10);
  targetDistPublisher = n.advertise<std_msgs::Float32>("/" + controller->getRobotNs() + "/distance", 10);

  this->setSubscriber();
}

void ImpedanceController::getParam() {
  std::vector<std::string> targetTopicName_;
  if (n.getParam("target_topic", targetTopicName_))
    targetName = targetTopicName_[controller->getIndex()];
  else if (!n.getParam("target_topic", targetName))
    ROS_ERROR_STREAM("target pose topic name(s) are not configured");

  if (!n.getParam("error_threshold/target/pos", posThr))
    ROS_ERROR_STREAM("error_threshold/target/pos is not configured");

  this->setSubscriber();
  controller->updateFilterCutoff(10.0, 10.0);
 
  if (!n.getParam("error_threshold/target/vel", velThr))
    ROS_ERROR_STREAM("error_threshold/target/vel is not configured");

  if (!n.getParam("error_threshold/target/force", forceThr))
    ROS_ERROR_STREAM("error_threshold/target/force is not configured");

  if (!n.getParam("error_threshold/rest/pos", posThr_r))
    ROS_ERROR_STREAM("error_threshold/rest/pos is not configured");

  if (!n.getParam("error_threshold/rest/vel", velThr_r))
    ROS_ERROR_STREAM("error_threshold/rest/vel is not configured");

  if (!n.getParam("error_threshold/rest/force", forceThr_r))
    ROS_ERROR_STREAM("error_threshold/rest/force is not configured");

  if (!n.getParam("limit/time", timeLimit))
    ROS_ERROR_STREAM("time limit is not configured");

  if (!n.getParam("limit/force", forceLimit))
    ROS_ERROR_STREAM("force limit is not configured");

  if (!n.getParam("target_rule/repeat", repeat))
    ROS_ERROR_STREAM("repeat is not configured");

  if (!n.getParam("target_rule/rest_everytime", restEverytime))
    ROS_ERROR_STREAM("restEverytime is not configured");

  ROS_INFO_STREAM("error_threshold/target/[pos, vel, force]: [" << posThr << ", " << velThr << ", " << forceThr << "]");
  ROS_INFO_STREAM("error_threshold/rest/[pos, vel, force]: [" << posThr_r << ", " << velThr_r << ", " << forceThr_r << "]");
}

void ImpedanceController::setSubscriber() {
  targetSubscriber = n.subscribe<geometry_msgs::PoseArray>(targetName, 1, &ImpedanceController::cbTargetPoses, this, th);
}

void ImpedanceController::getCriticalDampingCoeff(ImpCoeff& impCoeff, const std::vector<bool>& isGotMDK) {
  if (isGotMDK[0] == false) {
    impCoeff.d = Map<Vector3d>(impCoeff.d_.data());
    impCoeff.k = Map<Vector3d>(impCoeff.k_.data());
    impCoeff.m = impCoeff.d.array() * impCoeff.d.array() / impCoeff.k.array() * 0.25;
  } else if (isGotMDK[1] == false) {
    impCoeff.m = Map<Vector3d>(impCoeff.m_.data());
    impCoeff.k = Map<Vector3d>(impCoeff.k_.data());
    impCoeff.d = 2.0 * (impCoeff.m.array() * impCoeff.k.array()).cwiseSqrt();
  } else if (isGotMDK[2] == false) {
    impCoeff.m = Map<Vector3d>(impCoeff.m_.data());
    impCoeff.d = Map<Vector3d>(impCoeff.d_.data());
    impCoeff.k = impCoeff.d.array() * impCoeff.d.array() / impCoeff.m.array() * 0.25;
  }
}

ImpedanceController::ImpCoeff ImpedanceController::getImpCoeff() {
  ImpCoeff impCoeff;
  std::vector<bool> isGotMDK(3, false);
  isGotMDK[0] = n.getParam("imp_ceff/m", impCoeff.m_);
  isGotMDK[1] = n.getParam("imp_ceff/d", impCoeff.d_);
  isGotMDK[2] = n.getParam("imp_ceff/k", impCoeff.k_);

  int nGotMDK = std::accumulate(isGotMDK.begin(), isGotMDK.end(), 0);

  if (nGotMDK == 0) {
    isGotMDK[0] = n.getParam("imp_ceff_" + std::to_string(controller->getIndex()) + "/m", impCoeff.m_);
    isGotMDK[1] = n.getParam("imp_ceff_" + std::to_string(controller->getIndex()) + "/d", impCoeff.d_);
    isGotMDK[2] = n.getParam("imp_ceff_" + std::to_string(controller->getIndex()) + "/k", impCoeff.k_);
    nGotMDK = std::accumulate(isGotMDK.begin(), isGotMDK.end(), 0);
  }

  if (nGotMDK == 3) {
    impCoeff.m = Map<Vector3d>(impCoeff.m_.data());
    impCoeff.d = Map<Vector3d>(impCoeff.d_.data());
    impCoeff.k = Map<Vector3d>(impCoeff.k_.data());
  } else if (nGotMDK == 2) {
    ROS_INFO_STREAM("two of imp coeffs are configured. The last one is selected to achieve critical damping.");
    this->getCriticalDampingCoeff(impCoeff, isGotMDK);
  } else if (nGotMDK < 2) {
    ROS_ERROR_STREAM("al least, two of imp coeff is not configured");
    ros::shutdown();
  }

  return impCoeff;
}

ImpedanceController::ImpParam ImpedanceController::getImpParam(const ImpCoeff& impCoeff) {
  ROS_INFO_STREAM("Imp Coeff: m:[ " << impCoeff.m.transpose() << " ], d:[ " << impCoeff.d.transpose() << " ], k:[ " << impCoeff.k.transpose() << " ]");

  MatrixXd M_inv = impCoeff.m.cwiseInverse().asDiagonal(), D = impCoeff.d.asDiagonal(), K = impCoeff.k.asDiagonal();

  ImpParam impParam;
  impParam.A.block(0, 3, 3, 3) = Matrix3d::Identity();
  impParam.A.block(3, 0, 3, 3) = -M_inv * K;
  impParam.A.block(3, 3, 3, 3) = -M_inv * D;

  impParam.B.block(3, 0, 3, 3) = M_inv;

  impParam.C.block(3, 0, 3, 3) = M_inv * K;
  impParam.C.block(3, 3, 3, 3) = M_inv * D;

  return impParam;
}

void ImpedanceController::cbTargetPoses(const geometry_msgs::PoseArray::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_imp);
  tf2::fromMsg(*msg, _targetPoses);
  this->_targetUpdated = true;
}

bool ImpedanceController::NormReachedCheck(const VectorXd& delta_x, const VectorXd& force, const double posThr, const double velThr, const double forceThr) {
  // std::cout << delta_x.head(3).norm() << ", " << delta_x.tail(3).norm() << ", " << force.head(3).norm() << std::endl;
  // std::cout << posThr << ", " << velThr << ", " << forceThr << std::endl;
  // std::cout << "---" << std::endl;
  return (delta_x.head(3).norm() < posThr && delta_x.tail(3).norm() < velThr && force.head(3).norm() > forceThr);
}

// check if the robot eef reached the target pose
// delta_x: position error (3) and velocity error (3)
bool ImpedanceController::checkTargetReached(const VectorXd& delta_x, const VectorXd& force) {
  return NormReachedCheck(delta_x, force, this->posThr, this->velThr, this->forceThr);
}

bool ImpedanceController::checkRestTargetReached(const VectorXd& delta_x, const VectorXd& force) {
  return NormReachedCheck(delta_x, force, this->posThr_r, this->velThr_r, this->forceThr_r);
}

TaskState ImpedanceController::updataTaskState(const VectorXd& delta_x, const int targetIdx) {
  TaskState taskState = TaskState::OnGoing;

  VectorXd force = tf2::fromMsg(controller->getForceEef().wrench).head(3);

  // check if the robot eef reached the target pose
  if (targetIdx == -1) {  // if going back to rest positon
    if (checkRestTargetReached(delta_x, force) && !blocked)
      taskState = TaskState::Success;
  } else {
    if (checkTargetReached(delta_x, force)) {
      RespawnReqPublisher.publish(std_msgs::Empty());
      nCompletedTask++;
      taskState = TaskState::Success;
      ROS_INFO_STREAM(controller->getRobotNs() + " reached the target #" << targetIdx << " pose (" << nCompletedTask << ") ");
    }
  }

  // check if the robot eef failed to reach the target pose
  if (taskState == TaskState::OnGoing && (getTrialTime().toSec() > this->timeLimit || force.norm() > this->forceLimit)) {
    // RespawnReqPublisher.publish(std_msgs::Empty());
    taskState = TaskState::Fail;
    ROS_ERROR_STREAM(controller->getRobotNs() + " failed to reach the target pose");
  }

  // rest timer
  if (taskState == TaskState::Success || taskState == TaskState::Fail)
    this->t_start = ros::Time::now();

  return taskState;
}

VectorXd ImpedanceController::getControlState(const VectorXd& x, const VectorXd& xd, const VectorXd& exForce, const double dt, const ImpParam& impParam) {
  return (MatrixXd::Identity(6, 6) + impParam.A * dt) * x + dt * impParam.B * exForce + dt * impParam.C * xd;
}

Affine3d ImpedanceController::getNextTarget(const TaskState& taskState, const std::vector<Affine3d>& targetPoses, const Affine3d& restPose, int& targetIdx, int& nextTargetIdx) {
  switch (taskState) {
    case TaskState::Fail:
      targetIdx = -1;
      break;

    case TaskState::Success:
      if (restEverytime ? (targetIdx == -1) && ((nextTargetIdx != targetPoses.size() - 1) || repeat) : true) {  // if going back to rest positon
        nextTargetIdx++;
        if (nextTargetIdx == targetPoses.size())
          nextTargetIdx = repeat ? 0 : targetPoses.size() - 1;
        targetIdx = nextTargetIdx;
      } else {
        targetIdx = -1;
      }
      break;
  }

  if (targetIdx == -1)  // if going back to rest positon
    return restPose;
  else
    return targetPoses[targetIdx];
}

bool ImpedanceController::updateImpedanceTarget(const VectorXd& x, VectorXd& xd) {
  std::vector<Affine3d> targetPoses;
  // subsribe target poses
  {
    std::lock_guard<std::mutex> lock(mtx_imp);
    if (!this->_targetUpdated)
      return false;
    targetPoses = _targetPoses;
  }

  // transform target pose coordinate from world to base
  Affine3d T_base_world_inv = controller->getT_base_world().inverse();
  for (int i = 0; i < targetPoses.size(); i++)
    targetPoses[i] = T_base_world_inv * targetPoses[i];

  // update task state
  this->taskState = updataTaskState(x - xd, targetIdx);

  // update target pose
  xd.head(3) = getNextTarget(this->taskState, targetPoses, restPose, targetIdx, nextTargetIdx).translation();

  // curTargetId = targetIdx;

  return true;
}

void ImpedanceController::updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) {
  KDL::Frame frame;
  KDL::Twist vel;
  controller->getCartState(frame, vel);

  if (taskState == TaskState::Initial) {
    this->restPose = controller->getT_init();

    x = (VectorXd(6) << frame.p[0], frame.p[1], frame.p[2], 0.0, 0.0, 0.0).finished();
    xd = (VectorXd(6) << restPose.translation(), Vector3d::Zero()).finished();

    taskState = TaskState::OnGoing;
    controller->startOperation();

    this->t_start = ros::Time::now();
  }

  // update only position using subscribed q.
  // When the velocity is updated as well, the robot motion somehow become unstable.
  x.head(3) << frame.p[0], frame.p[1], frame.p[2];

  // update target pose
  if (!this->updateImpedanceTarget(x, xd))
    xd.tail(3) = Vector3d::Zero();
  else
    taskState == TaskState::Initial;

  // get command state
  x = getControlState(x, xd, tf2::fromMsg(controller->getForceEef().wrench).head(3), controller->dt, this->impParam);
  // std::cout << " x: " << x.transpose() << std::endl;
  // std::cout << "xd: " << xd.transpose() << std::endl;

  tf::vectorEigenToKDL(x.head(3), pose.p);
  tf::vectorEigenToKDL(x.tail(3), twist.vel);

  this->targetDistance = (x - xd).head(3).norm();  // TODO: generalize this

  std_msgs::Float32 msg;
  msg.data = targetDistance;
  targetDistPublisher.publish(msg);
}
