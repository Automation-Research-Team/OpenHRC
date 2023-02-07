#include "ohrc_control/admittance_controller.hpp"

AdmittanceController::AdmittanceController() {
  subForce = nh.subscribe<geometry_msgs::WrenchStamped>("/" + robot_ns + "ft_sensor/filtered", 2, &AdmittanceController::cbForce, this, th);
  this->Tft_eff = trans.getTransform(robot_ns + chain_end, robot_ns + "ft_sensor_link", ros::Time(0), ros::Duration(1.0));

  pubPoint = nh.advertise<geometry_msgs::PointStamped>("/point", 1);

  if (!initAdmittanceModel())
    ros::shutdown();
}

void AdmittanceController::cbForce(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx);
  _force = *msg;

  if (!flagEffPose)
    flagEffPose = true;
}

bool AdmittanceController::initAdmittanceModel(std::vector<double> m, std::vector<double> d, std::vector<double> k, std::vector<double> i, std::vector<double> do_,
                                               std::vector<double> ko) {
  ROS_INFO_STREAM("Adminntace translation: m [" << Vector3d(m.data()).transpose() << "], d [" << Vector3d(d.data()).transpose() << "], k [" << Vector3d(k.data()).transpose()
                                                << "]");
  ROS_INFO_STREAM("Adminntace rotation: i [" << Vector3d(i.data()).transpose() << "], do [" << Vector3d(do_.data()).transpose() << "], ko [" << Vector3d(ko.data()).transpose()
                                             << "]");

  if ((Array3d(m.data()) < 1.0e-3).any() || (Array3d(i.data()) < 1.0e-3).any()) {
    ROS_FATAL("Mass and inertia coefficients should be more than zero!");
    return false;
  }

  MatrixXd M_inv = Vector3d(m.data()).asDiagonal().inverse(), D = Vector3d(d.data()).asDiagonal(), K = Vector3d(k.data()).asDiagonal();

  A.block(0, 3, 3, 3) = Matrix3d::Identity();
  A.block(3, 0, 3, 3) = -M_inv * K;
  A.block(3, 3, 3, 3) = -M_inv * D;

  B.block(3, 0, 3, 3) = M_inv;

  C.block(3, 0, 3, 3) = M_inv * K;

  I = Vector3d(i.data()).asDiagonal();
  Do = Vector3d(do_.data()).asDiagonal();
  Ko = Vector3d(ko.data()).asDiagonal();

  return true;
}

bool AdmittanceController::initAdmittanceModel() {
  std::vector<double> m(3), d(3), k(3);
  nh.getParam("admittance/trans/m", m);
  nh.getParam("admittance/trans/d", d);
  nh.getParam("admittance/trans/k", k);

  std::vector<double> i(3), do_(3), ko(3);
  nh.getParam("admittance/rotation/m", i);
  nh.getParam("admittance/rotation/d", do_);
  nh.getParam("admittance/rotation/k", ko);

  return initAdmittanceModel(m, d, k, i, do_, ko);
}

void AdmittanceController::resetFt() {
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/ft_sensor_filter/reset_offset");
  std_srvs::Empty srv;
  client.call(srv);
}

void AdmittanceController::initWithJnt(const KDL::JntArray& q_init) {
  this->resetFt();
}

void AdmittanceController::getDesEffPoseVel(const double& dt, const KDL::JntArray& q_cur, const KDL::JntArray& dq_cur, KDL::Frame& des_eff_pose, KDL::Twist& des_eff_vel) {
  geometry_msgs::WrenchStamped ft_msg;
  Affine3d Td;
  {  // target pose & vel coming from another interface
    std::lock_guard<std::mutex> lock(mtx);

    tf::transformKDLToEigen(this->_des_eff_pose, Td);
    // des_eff_vel = this->_des_eff_vel; // TODO: consider target velocity
    ft_msg = this->_force;
  }

  KDL::Frame cart;
  fk_solver_ptr->JntToCart(q_cur, cart);
  Affine3d Teff_base;
  tf::transformKDLToEigen(cart, Teff_base);

  VectorXd ft = tf2::fromMsg(ft_msg.wrench);
  VectorXd ft_eff = trans.transformFT(ft, Tft_eff.inverse());
  VectorXd ft_eff_base(6);
  ft_eff_base.head(3) = Teff_base.rotation() * ft_eff.head(3);
  ft_eff_base.tail(3) = Teff_base.rotation() * ft_eff.tail(3);

  static VectorXd x = (VectorXd(6) << Teff_base.translation(), VectorXd::Zero(3)).finished();
  static Quaterniond quat = Quaterniond(Teff_base.rotation());
  static Vector3d omega = Vector3d::Zero();

  // translation
  Vector3d delta_x = Td.translation();
  x = (MatrixXd::Identity(6, 6) + dt * A) * x + dt * B * ft_eff_base.head(3) + dt * C * delta_x;

  // Rotation
  Vector3d delta_rot = rotation_util::getRotationMatrixError(quat.matrix(), Td.rotation());
  omega = omega + dt * I.inverse() * (-Do * omega - Ko * delta_rot + ft_eff_base.tail(3));

  Vector4d dQuat;                                                      // (x,y,z,w)
  dQuat.head(3) = 0.5 * (quat.w() * omega - quat.vec().cross(omega));  // Handbook of Robotics (Siciliano, 2nd Ed.) below eq.(2.8)
  dQuat(3) = -0.5 * omega.transpose() * quat.vec();

  Vector4d newQaut = (quat.coeffs() + dQuat * dt).normalized();                                                // (x,y,z,w)
  quat = rotation_util::checkFlipQuaternionSign(Quaterniond(newQaut(3), newQaut(0), newQaut(1), newQaut(2)));  // Quaterniond(w, x, y, z)

  // static Affine3d Teff_base_init = Teff_base; // initial state
  // Affine3d Td_eff_base = Translation3d(x.head(3)) * Teff_base_init.rotation(); // translation only
  // Affine3d Td_eff_base = Translation3d(Teff_base_init.translation()) * quat;  // rotation only
  Affine3d Td_eff_base = Translation3d(x.head(3)) * quat;  // translation and rotation
  tf::transformEigenToKDL(Td_eff_base, des_eff_pose);

  // velocity
  VectorXd v_eff_base = VectorXd::Zero(6);
  v_eff_base.head(3) = x.tail(3);  // translation velocity
  v_eff_base.tail(3) = omega;      // rotation velocity
  tf::twistEigenToKDL(v_eff_base, des_eff_vel);
}
