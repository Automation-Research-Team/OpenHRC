#include "ohrc_automation/impedance_controller.hpp"

ImpedanceController::ImpedanceController() {
  Vector3d m(1., 1., 1.), k(15., 15., 5.);
  this->impParam = getImpParam(m, k);
  //   Affine3d restPoseOffset = Affine3d::Iden;
}

ImpedanceController::ImpParam ImpedanceController::getImpParam(const Vector3d& m, const Vector3d& k) {
  Vector3d d = 2.0 * (m.array() * k.array()).cwiseSqrt();
  MatrixXd M_inv = m.cwiseInverse().asDiagonal(), D = d.asDiagonal(), K = k.asDiagonal();

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
  tf2::fromMsg(*msg, targetPoses);
  this->_targetUpdated = true;
}

ImpedanceController::TaskState ImpedanceController::updataTaskState(const int targetIdx, CartController* controller) {
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

void ImpedanceController::updateTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) {
  static Affine3d curTargetPose;
  if (taskState == TaskState::Initial) {
    // Affine3d restPoseOffset = ;
    restPose = controller->getT_init();
    restPose.translation()[0] += 0.1;
    curTargetPose = restPose;

    targetIdx = -1;

    controller->startOperation();
  }

  taskState = updataTaskState(targetIdx, controller);

  KDL::Frame frame;
  KDL::Twist vel;
  controller->getCartState(frame, vel);
  // VectorXd x2 = (VectorXd(6) << frame.p[0], frame.p[1], frame.p[2], vel.vel[0], vel.vel[1], vel.vel[2]).finished();
  // VectorXd x = (VectorXd(6) << frame.p[0], frame.p[1], frame.p[2], vel.vel[0], vel.vel[1], vel.vel[2]).finished();
  static VectorXd x = (VectorXd(6) << frame.p[0], frame.p[1], frame.p[2], 0.0, 0.0, 0.0).finished();
  x.head(3) << frame.p[0], frame.p[1], frame.p[2];
  VectorXd xd = (VectorXd(6) << curTargetPose.translation(), Vector3d::Zero()).finished();
  VectorXd exForce = VectorXd::Zero(3);

  x = getControlState(x, xd, exForce, dt);

  tf::vectorEigenToKDL(x.head(3), pose.p);
  tf::vectorEigenToKDL(x.tail(3), twist.vel);
}
