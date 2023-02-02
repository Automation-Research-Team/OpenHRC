#include "ohrc_automation/impedance_controller.hpp"

ImpedanceController::ImpedanceController()
{
  Vector3d m(1, 1, 1), k(10, 10, 5);
  this->impParam = getImpParam(m, k);

  //   Affine3d restPoseOffset = Affine3d::Iden;
}

ImpedanceController::ImpParam ImpedanceController::getImpParam(const Vector3d& m, const Vector3d& k)
{
  Vector3d d = 2.0 * (m.array() * k.array()).cwiseSqrt();
  MatrixXd M_inv = m.asDiagonal().inverse(), D = d.asDiagonal(), K = k.asDiagonal();

  ImpParam impParam;
  impParam.A.block(0, 3, 3, 3) = Matrix3d::Identity();
  impParam.A.block(3, 0, 3, 3) = -M_inv * K;
  impParam.A.block(3, 3, 3, 3) = -M_inv * D;

  impParam.B.block(3, 0, 3, 3) = M_inv;

  impParam.C.block(3, 0, 3, 3) = M_inv * K;
  impParam.C.block(3, 3, 3, 3) = M_inv * D;

  return impParam;
}

void ImpedanceController::cbTargetPoses(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mtx_imp);
  tf2::fromMsg(*msg, targetPoses);
  this->_targetUpdated = true;
}

ImpedanceController::TaskState ImpedanceController::updataTaskState(const int targetIdx, CartController* controller)
{
  return TaskState::OnGoing;
}

VectorXd ImpedanceController::impedanceController(const VectorXd& x, const VectorXd& xd, const VectorXd& exForce,
                                                  const double dt)
{
  //   // std::cout <<   param.targetType << std::endl;
  //   // std::cout << magic_enum::enum_name(adaptationOption) << std::endl;
  //   if (adaptationOption == AdaptationOption::random_multi && param.targetType == "Automation")
  //     return (MatrixXd::Identity(6, 6) + param.impParam_v[param.d_thr_i].A * dt) * x +
  //            dt * param.impParam_v[param.d_thr_i].C * param.xd;
  //   else
  return (MatrixXd::Identity(6, 6) + impParam.A * dt) * x + dt * impParam.B * exForce + dt * impParam.C * param.xd;
}

void ImpedanceController::updateTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller)
{
  if (taskState == TaskState::Initial)
  {
    // Affine3d restPoseOffset;
    restPose = controller->getT_init();  // * restPoseOffset;
    Affine3d curTargetPose = restPose;

    targetIdx = -1;
  }

  taskState = updataTaskState(targetIdx, controller);

  getControlState(x, xd, exForce, dt);
}
