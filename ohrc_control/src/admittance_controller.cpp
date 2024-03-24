#include "ohrc_control/admittance_controller.hpp"

void AdmittanceController::initInterface() {
  this->impParam = getImpParam(getImpCoeff());
}

void AdmittanceController::getCriticalDampingCoeff(ImpCoeff& impCoeff, const std::vector<bool>& isGotMDK) {
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

AdmittanceController::ImpCoeff AdmittanceController::getImpCoeff() {
  ImpCoeff impCoeff;
  std::vector<bool> isGotMDK(3, false);
  isGotMDK[0] = n.getParam("/imp_ceff/m", impCoeff.m_);
  isGotMDK[1] = n.getParam("/imp_ceff/d", impCoeff.d_);
  isGotMDK[2] = n.getParam("/imp_ceff/k", impCoeff.k_);

  int nGotMDK = std::accumulate(isGotMDK.begin(), isGotMDK.end(), 0);

  if (nGotMDK == 0) {
    isGotMDK[0] = n.getParam("/imp_ceff_" + std::to_string(controller->getIndex()) + "/m", impCoeff.m_);
    isGotMDK[1] = n.getParam("/imp_ceff_" + std::to_string(controller->getIndex()) + "/d", impCoeff.d_);
    isGotMDK[2] = n.getParam("/imp_ceff_" + std::to_string(controller->getIndex()) + "/k", impCoeff.k_);
    nGotMDK = std::accumulate(isGotMDK.begin(), isGotMDK.end(), 0);
  }

  if (nGotMDK == 3) {
    impCoeff.m = Map<Vector3d>(impCoeff.m_.data());
    impCoeff.d = Map<Vector3d>(impCoeff.d_.data());
    impCoeff.k = Map<Vector3d>(impCoeff.k_.data());

    skipMass = (impCoeff.m.array() < 1.0e-2).all();
      
  } else if (nGotMDK == 2) {
    ROS_INFO_STREAM("two of imp coeffs are configured. The last one is selected to achieve critical damping.");
    this->getCriticalDampingCoeff(impCoeff, isGotMDK);
  } else if (nGotMDK < 2) {
    ROS_ERROR_STREAM("al least, two of imp coeff is not configured");
    ros::shutdown();
  }

  return impCoeff;
}

AdmittanceController::ImpParam AdmittanceController::getImpParam(const ImpCoeff& impCoeff) {
  ROS_INFO_STREAM("Imp Coeff: m:[ " << impCoeff.m.transpose() << " ], d:[ " << impCoeff.d.transpose() << " ], k:[ " << impCoeff.k.transpose() << " ]");

  

  ImpParam impParam;
  if (skipMass) {
    MatrixXd D_inv = impCoeff.d.cwiseInverse().asDiagonal(), K = impCoeff.k.asDiagonal();
    f_dx = [K, D_inv](const double& t, const VectorXd& x, const std::vector<VectorXd>& u) { return -K * D_inv * x + D_inv * u[0] + K * D_inv * u[1] + u[2]; };

  }else{
    MatrixXd M_inv = impCoeff.m.cwiseInverse().asDiagonal(), D = impCoeff.d.asDiagonal(), K = impCoeff.k.asDiagonal();
    MatrixXd A = MatrixXd::Zero(6, 6), B = MatrixXd::Zero(6, 3), C = MatrixXd::Zero(6, 6);
    A.block(0, 3, 3, 3) = Matrix3d::Identity();
    A.block(3, 0, 3, 3) = -M_inv * K;
    A.block(3, 3, 3, 3) = -M_inv * D;

    B.block(3, 0, 3, 3) = M_inv;

    C.block(3, 0, 3, 3) = M_inv * K;
    C.block(3, 3, 3, 3) = M_inv * D;

    f_dx = [A, B, C](const double& t, const VectorXd& x, const std::vector<VectorXd>& u) { return A * x + B * u[0] + C * u[1]; };
  }
  integrator.reset(new math_utility::Integrator(math_utility::Integrator::Method::RungeKutta, f_dx, dt));
  return impParam;
}

VectorXd AdmittanceController::getControlState(const VectorXd& x, const VectorXd& xd, const VectorXd& exForce, const double dt, const ImpParam& impParam) {
  if (skipMass) {
    VectorXd x_ = integrator->integrate(0.0, x.head(3), {exForce, xd.head(3), xd.tail(3)});
    return (VectorXd(6) << x_, f_dx(0.0, x_, {exForce, xd.head(3), xd.tail(3)})).finished();

  }else
    return integrator->integrate(0.0, x, {exForce, xd}); //impParam.A * x + impParam.B * exForce + impParam.C * xd;
}

void AdmittanceController::updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) {
  KDL::Frame frame;
  KDL::Twist vel;
  controller->getCartState(frame, vel);

  if (taskState == TaskState::Initial) {
    this->restPose = controller->getT_init();

    x = (VectorXd(6) << frame.p[0], frame.p[1], frame.p[2], 0.0, 0.0, 0.0).finished();
    xd = (VectorXd(6) << restPose.translation(), Vector3d::Zero()).finished();

    taskState = TaskState::OnGoing;
    // controller->startOperation();
  }

  // update only position using subscribed q.
  // When the velocity is updated as well, the robot motion somehow become unstable.
  x.head(3) << frame.p[0], frame.p[1], frame.p[2];

  // update target pose
  xd << pose.p.x(), pose.p.y(), pose.p.z(), twist.vel.x(), twist.vel.y(), twist.vel.z();

  // get command state
  x = getControlState(x, xd, tf2::fromMsg(controller->getForceEef().wrench).head(3), controller->dt, this->impParam);
  // std::cout << " x: " << x.transpose() << std::endl;
  // std::cout << "xd: " << xd.transpose() << std::endl;

  tf::vectorEigenToKDL(x.head(3), pose.p);
  tf::vectorEigenToKDL(x.tail(3), twist.vel);

  // this->targetDistance = (x - xd).head(3).norm();  // TODO: generalize this

  // std_msgs::Float32 msg;
  // msg.data = targetDistance;
  // targetDistPublisher.publish(msg);
}
