#include "ohrc_automation/impedance_controller.hpp"

ImpedanceController::ImpedanceController() {
  ros::NodeHandle n("~");

  // Vector3d m(1., 1., 1.), k(15., 15., 5.);
  ImpCoeff impCoeff;
  std::vector<bool> isGotMDK(3, false);
  isGotMDK[0] = n.getParam("imp_ceff/m", impCoeff.m_);
  isGotMDK[1] = n.getParam("imp_ceff/d", impCoeff.d_);
  isGotMDK[2] = n.getParam("imp_ceff/k", impCoeff.k_);

  int nGotMDK = std::accumulate(isGotMDK.begin(), isGotMDK.end(), 0);
  if (nGotMDK == 2) {
    ROS_INFO_STREAM("two of imp coeffs are configured. The last one is selected to achieve critical damping.");
    this->getCriticalDampingCoeff(impCoeff, isGotMDK);

  } else if (nGotMDK < 2) {
    ROS_ERROR_STREAM("al least, two of imp coeff is not configured");
    ros::shutdown();
  }
  this->impParam = getImpParam(impCoeff);

  std::string targetTopicName = "/gazebo/automation_targets";
  targetSubscriber = n.subscribe<geometry_msgs::PoseArray>(targetTopicName, 1, &ImpedanceController::cbTargetPoses, this);

  //   Affine3d restPoseOffset = Affine3d::Iden;
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

ImpedanceController::TaskState ImpedanceController::updataTaskState(const VectorXd& delta_x, CartController* controller) {
  if (delta_x.head(3).norm() < 0.01 && delta_x.tail(3).norm() < 0.01)
    return TaskState::Success;

  return TaskState::OnGoing;
}

VectorXd ImpedanceController::getControlState(const VectorXd& x, const VectorXd& xd, const VectorXd& exForce, const double dt) {
  //   // std::cout <<   param.targetType << std::endl;
  //   // std::cout << magic_enum::enum_name(adaptationOption) << std::endl;
  //   if (adaptationOption == AdaptationOption::random_multi && param.targetType == "Automation")
  //     return (MatrixXd::Identity(6, 6) + param.impParam_v[param.d_thr_i].A * dt) * x +
  //            dt * param.impParam_v[param.d_thr_i].C * param.xd;
  //   else
  return (MatrixXd::Identity(6, 6) + impParam.A * dt) * x + dt * impParam.B * exForce + dt * impParam.C * xd;
}

void ImpedanceController::getNextTarget(const ImpedanceController::TaskState& taskState, const std::vector<Affine3d>& targetPoses, const Affine3d& restPose, int& targetIdx,
                                        int& nextTargetIdx, Affine3d& curTargetPose) {
  if (taskState == TaskState::Fail)
    curTargetPose = restPose;
  else if (taskState == TaskState::Success) {
    if (targetIdx == -1) {
      nextTargetIdx++;
      if (nextTargetIdx == targetPoses.size())
        nextTargetIdx = 0;
      curTargetPose = targetPoses[nextTargetIdx];
      targetIdx = nextTargetIdx;
    } else {
      curTargetPose = restPose;
      targetIdx = -1;
    }
  }
}

void ImpedanceController::updateTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) {
  std::vector<Affine3d> targetPoses;
  {
    std::lock_guard<std::mutex> lock(mtx_imp);
    if (!this->_targetUpdated)
      return;
    targetPoses = _targetPoses;
  }

  static Affine3d curTargetPose;
  if (taskState == TaskState::Initial) {
    // _targetPoses.resize(1);
    // _targetPoses[0] = controller->getT_init();
    // _targetPoses[0].translation()[0] += 0.1;

    restPose = controller->getT_init();
    curTargetPose = restPose;
    targetIdx = -1;

    controller->startOperation();
  }

  KDL::Frame frame;
  KDL::Twist vel;
  controller->getCartState(frame, vel);
  static VectorXd x = (VectorXd(6) << frame.p[0], frame.p[1], frame.p[2], 0.0, 0.0, 0.0).finished();
  x.head(3) << frame.p[0], frame.p[1], frame.p[2];

  VectorXd xd = (VectorXd(6) << curTargetPose.translation(), Vector3d::Zero()).finished();

  taskState = updataTaskState(x - xd, controller);
  getNextTarget(taskState, targetPoses, restPose, targetIdx, nextTargetIdx, curTargetPose);

  // update target pose
  xd.head(3) = curTargetPose.translation();

  x = getControlState(x, xd, VectorXd::Zero(3), dt);

  tf::vectorEigenToKDL(x.head(3), pose.p);
  tf::vectorEigenToKDL(x.tail(3), twist.vel);
}
