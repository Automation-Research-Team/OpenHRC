#include "ohrc_control/omega_controller.hpp"

// OmegaController::OmegaController(std::string robot) : CartController(robot) {
//   init();
// }

OmegaController::OmegaController() : CartController() {
  init();
}

void OmegaController::init() {
  this->Tft_eff = trans.getTransform(robot_ns + chain_end, robot_ns + "eff_hook", ros::Time(0), ros::Duration(1.0));
  this->Tomega_base = trans.getTransform(robot_ns + chain_start, "omega_link", ros::Time(0), ros::Duration(1.0));

  std::string omega, haptic;
  nh.param("omega_type", omega, std::string("left"));
  nh.param("haptic_type", haptic, std::string("None"));
  ROS_INFO_STREAM("Omega type: " << omega << ", Haptic type: " << haptic);

  subOmega = nh.subscribe<omega_haptic_device::Omega>("/omega_driver/" + omega + "/state", 2, &OmegaController::cbOmegaState, this, th);
  pubOmegaForce = nh.advertise<geometry_msgs::Wrench>("/omega_driver/" + omega + "/cmd_force", 2);

  subLeaderEnergy = nh.subscribe<std_msgs::Float32>("/passivity_observer/leader/energy", 2, &OmegaController::cbEnergy, this, th);
  pubEnergy = nh.advertise<std_msgs::Float32>("/passivity_observer/follower/energy", 2);

  if (haptic == "PositionPositionFeedback")
    hapticType = HapticType::PositionPositionFeedback;
  else if (haptic == "PositionForce")
    hapticType = HapticType::PositionForce;
  else if (haptic == "PositionForceFeedback")
    hapticType = HapticType::PositionForceFeedback;
  else
    hapticType = HapticType::None;

  if (hapticType == HapticType::PositionForce || hapticType == HapticType::PositionForceFeedback) {
    subForce = nh.subscribe<geometry_msgs::WrenchStamped>("/" + robot_ns + "ft_sensor/filtered", 2, &OmegaController::cbForce, this, th);
    subFlagPtrs.push_back(&flagForce);
  }
}

void OmegaController::cbForce(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx);
  _force = *msg;

  if (!flagForce)
    flagForce = true;
}

void OmegaController::cbEnergy(const std_msgs::Float32::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx);
  _leaderEnergy = msg->data;
}

void OmegaController::cbOmegaState(const omega_haptic_device::Omega::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx);
  if (!flagJntState)  // This waits for initilizing _des_eff_pose
    return;

  double k_trans = 2.0;
  Matrix3d R = Tomega_base.rotation();
  Affine3d T_omega = Translation3d(k_trans * R * Vector3d(-msg->pose.position.x, -msg->pose.position.y, msg->pose.position.z)) *
                     (R * Quaterniond(msg->pose.orientation.w, -msg->pose.orientation.x, -msg->pose.orientation.y, msg->pose.orientation.z));
  VectorXd v_omega = (VectorXd(6) << k_trans * R * Vector3d(-msg->twist.linear.x, -msg->twist.linear.y, msg->twist.linear.z), R * Vector3d(-msg->twist.angular.x, -msg->twist.angular.y, msg->twist.angular.z)).finished();

  static Affine3d T = T_init, T_start = T_init, T_omega_start = T_omega;
  VectorXd v = VectorXd::Zero(6);
  if (msg->gripper.button) {
    T = Translation3d(T_omega.translation() - T_omega_start.translation() + T_start.translation()) * (T_omega.rotation() * T_init.rotation() * R);  // TODO: correct???
    v = v_omega;
    _disable = false;
  } else {
    T_start = T;
    T_omega_start = T_omega;
    v = v * 0.9;  // TODO: stop smoothly

    _disable = true;
  }
  VectorXd ft = (VectorXd(6) << R * Vector3d(-msg->wrench.force.x, -msg->wrench.force.y, msg->wrench.force.z), R * Vector3d(-msg->wrench.torque.x, -msg->wrench.torque.y, msg->wrench.torque.z)).finished();

  tf::transformEigenToKDL(T, _des_eff_pose);
  tf::twistEigenToKDL(v, _des_eff_vel);

  _force_omega.header.stamp = msg->header.stamp;
  _force_omega.header.frame_id = "base";
  tf2::toMsg(ft, _force_omega.wrench);

  if (!flagEffPose)
    flagEffPose = true;
}

void OmegaController::initWithJnt(const KDL::JntArray& q_init) {
  if (hapticType != HapticType::None) {
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/" + robot_ns + "ft_sensor/reset_offset");
    std_srvs::Empty srv;
    client.call(srv);
  }
}

void OmegaController::getDesEffPoseVel(const double& dt, const KDL::JntArray& q_cur, const KDL::JntArray& dq_cur, KDL::Frame& des_eff_pose, KDL::Twist& des_eff_vel) {
  geometry_msgs::WrenchStamped ft_msg, ft_omega_msg, ft_assist_msg;
  double leaderEnergy = 0.0;
  {  // target pose & vel coming from another interface
    std::lock_guard<std::mutex> lock(mtx);
    // des_eff_pose = this->_des_eff_pose;
    // des_eff_vel = this->_des_eff_vel;
    ft_omega_msg = this->_force_omega;
    ft_msg = this->_force;
    ft_assist_msg = this->_force_assist;
    leaderEnergy = this->_leaderEnergy;
  }

  CartController::getDesEffPoseVel(dt, q_cur, dq_cur, des_eff_pose, des_eff_vel);

  KDL::Frame cart;
  fk_solver_ptr->JntToCart(q_cur, cart);
  Affine3d Teff_base;
  tf::transformKDLToEigen(cart, Teff_base);
  static Affine3d Teff_base_init = Teff_base;

  Affine3d Tft_base = Teff_base * Tft_eff;

  VectorXd ft = tf2::fromMsg(ft_msg.wrench);
  VectorXd ft_base = trans.transformFT(ft, Tft_base.inverse());

  Vector3d v;
  tf::vectorKDLToEigen(des_eff_vel.vel, v);

  VectorXd ft_omega = tf2::fromMsg(ft_omega_msg.wrench);

  VectorXd ft_feedback = tf2::fromMsg(ft_assist_msg.wrench);
  double k_force = 0.0;
  Affine3d T;
  switch (hapticType) {
    case HapticType::PositionPositionFeedback:  // position-positon feedback control
      tf::transformKDLToEigen(des_eff_pose, T);
      ft_feedback.head(3) += Teff_base.translation() - T.translation();
      k_force = 100.0;
      break;

    case HapticType::PositionForce:  // force reflection type (a.k.a. position-force control)
      ft_feedback += ft_base;
      k_force = 0.2;
      break;

    case HapticType::PositionForceFeedback:
      ft_feedback += ft_base - ft_omega;  // force feedback type (a.k.a. position-force feedback control)
      k_force = 0.2;
      break;

    default:
      break;
  }

  ft_feedback = ft_feedback * k_force;

  static Eigen::Vector3d E_out = Eigen::Vector3d::Zero();
  static Eigen::Vector3d E_PC = Eigen::Vector3d::Zero();

  int i = 2;
  // for (int i = 0; i < 3; i++) {
  double E_in = 0.0;  // leaderEnergy;

  double E_PO = 0.0;

  E_out(i) += v(i) * ft_feedback(i) * dt;
  E_PO = E_in + E_out(i) + E_PC(i);

  // std::cout << E_PO << std::endl;

  double beta = 0.0;
  if (E_PO < .0 && abs(v(i)) > 1.0E-4)
    beta = -E_PO / (dt * ft_feedback(i) * ft_feedback(i));

  // std_msgs::Float32 e;
  // e.data = E_out * k_force * 0.5;
  // pubEnergy.publish(e);

  // if (beta > 0.0) {
  //   std::cout << "E_PO: " << E_PO << ", E_out: " << v(2) * ft_feedback(2) << std::endl;
  //   std::cout << "(original vel) " << v(2) << " + (modification) " << beta * ft_feedback(2) << std::endl;
  // }

  // des_eff_vel.vel(i) = v(i) + beta * ft_feedback(i);

  E_PC(i) += beta * ft_feedback(i) * ft_feedback(i) * dt;
  // }
  geometry_msgs::Wrench wrench_omega;
  wrench_omega = tf2::toMsg(ft_feedback, wrench_omega);
  pubOmegaForce.publish(wrench_omega);
}