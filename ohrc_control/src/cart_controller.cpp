#include "ohrc_control/cart_controller.hpp"

CartController::CartController(const std::string robot, const std::string root_frame, const int index) : nh("~"), spinner(0), root_frame(root_frame), index(index) {
  // chain_start = root_frame;
  init(robot);
}

CartController::CartController(const std::string robot, const std::string root_frame) : nh("~"), spinner(0), root_frame(root_frame) {
  // chain_start = root_frame;
  init(robot);
}

CartController::CartController() : nh("~"), spinner(0) {
  std::string robot;
  nh.param("robot_ns", robot, std::string(""));
  root_frame = "world";
  // nh.param("chain_start", chain_start, std::string(""));
  init(robot);
}

void CartController::init(std::string robot) {
  std::signal(SIGINT, CartController::signal_handler);

  // nh.param("num_samples", num_samples, 1000);
  nh.param("chain_start", chain_start, std::string(""));
  if (root_frame == "")
    root_frame = chain_start;

  nh.param("chain_end", chain_end, std::string(""));

  nh.param("useManipulabilityOpt", useManipOpt, false);

  if (chain_start == "" || chain_end == "") {
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }

  timeout = 1.0 / freq;

  ROS_INFO_STREAM("name space: " + robot);
  if (robot_ns != "")
    robot_ns = robot + "/";
  urdf_param = "/" + robot_ns + "robot_description";

  this->T_base_root = trans.getTransform(root_frame, robot_ns + chain_start, ros::Time(0), ros::Duration(10.0));
  // this->Tft_eff = trans.getTransform(robot_ns + chain_end, robot_ns + "ft_sensor_link", ros::Time(0), ros::Duration(10.0));

  tracik_solver_ptr.reset(new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param, timeout, eps));

  KDL::JntArray ll, ul;  // lower joint limits, upper joint limits
  bool valid = tracik_solver_ptr->getKDLLimits(ll, ul);

  fk_solver_ptr.reset(new KDL::ChainFkSolverPos_recursive(chain));

  vik_solver_ptr.reset(new KDL::ChainIkSolverVel_pinv(chain));
  kdl_solver_ptr.reset(new KDL::ChainIkSolverPos_NR_JL(chain, ll, ul, *fk_solver_ptr, *vik_solver_ptr, 1, eps));

  myik_solver_ptr.reset(new MyIK::MyIK(chain_start, chain_end, urdf_param, eps, T_base_root));
  valid = myik_solver_ptr->getKDLChain(chain);
  chain_segs = chain.segments;

  nJnt = chain.getNrOfJoints();
  _q_cur.resize(nJnt);

  jntStateSubscriber = nh.subscribe("/" + robot_ns + "joint_states", 2, &CartController::cbJntState, this, th);
  subFlagPtrs.push_back(&flagJntState);

  if (useManipOpt) {
    userArmMarker = nh.subscribe("/arm_marker", 2, &CartController::cbArmMarker, this, th);
    subFlagPtrs.push_back(&flagArmMarker);
  }

  // subForce = nh.subscribe<geometry_msgs::WrenchStamped>("/" + robot_ns + "ft_sensor/filtered", 2, &CartController::cbForce, this, th);

  jntPosCmdPublisher = nh.advertise<std_msgs::Float64MultiArray>("/" + robot_ns + "joint_position_controller/command", 2);
  jntVelCmdPublisher = nh.advertise<std_msgs::Float64MultiArray>("/" + robot_ns + "joint_velocity_controller/command", 2);
  desStatePublisher = nh.advertise<ohrc_control::StateStamped>("/" + robot_ns + "desired_pose", 2);
  markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/" + robot_ns + "markers", 2);

  std::string solver_str;
  if (!nh.param("solver", solver_str, std::string("MyIK")))
    ROS_WARN_STREAM("Solver type is not choisen {Trac_Ik, KDL, MyIK}: Default MyIK");
  else
    ROS_INFO_STREAM("Solver: " << solver_str);

  solver = magic_enum::enum_cast<SolverType>(solver_str).value_or(SolverType::None);
  if (solver == SolverType::None) {
    ROS_FATAL("Solver type is not correctly choisen from {Trac_IK, KDL, MyIK}");
    exit(-1);
  }

  std::string controller_str;
  if (!nh.param("controller", controller_str, std::string("Position")))
    ROS_WARN_STREAM("Controller is not choisen {Position, Velocity, Torque}: Default Position");
  else
    ROS_INFO_STREAM("Controller: " << controller_str);

  controller = magic_enum::enum_cast<ControllerType>(controller_str).value_or(ControllerType::None);
  if (controller == ControllerType::None) {
    ROS_FATAL("Controller type is not correctly choisen from {Position, Velocity, Torque}");
    exit(-1);
  }

  // subFlagPtrs.push_back(&flagEffPose);

  std::vector<double> initial_pose;
  nh.param("initial_pose", initial_pose, std::vector<double>{ 0.45, 0.0, 0.85, 0.0, M_PI, -M_PI_2 });

  // std::cout << root_frame << std::endl;
  // std::cout << robot_ns + chain_start << std::endl;

  // T_init =
  //     Translation3d(initial_pose[0], initial_pose[1], initial_pose[2]) *
  //     (T_base_root.rotation() * (AngleAxisd(initial_pose[3], Vector3d::UnitX()) * AngleAxisd(initial_pose[4], Vector3d::UnitY()) * AngleAxisd(initial_pose[5],
  //     Vector3d::UnitZ())));
  T_init =
      Translation3d(initial_pose[0], initial_pose[1], initial_pose[2]) * (AngleAxisd(initial_pose[3], Vector3d::UnitX()) * AngleAxisd(initial_pose[4], Vector3d::UnitY()) * AngleAxisd(initial_pose[5], Vector3d::UnitZ()));

  for (int i = 0; i < 6; i++)
    velFilter.push_back(butterworth(2, 5.0, freq));

  for (int i = 0; i < nJnt; i++)
    jntFilter.push_back(butterworth(2, 5.0, freq));
}

CartController::~CartController() {
}

Affine3d CartController::getTransform_base(std::string target) {
  return trans.getTransform(robot_ns + chain_start, target, ros::Time(0), ros::Duration(1.0));
}

void CartController::signal_handler(int signum) {
  ros::NodeHandle nh;
  std::vector<std::string> robot_ns{ "/toroboarm_1", "/toroboarm_2" };
  std::vector<std::string> controller{ "/joint_position_controller", "/joint_velocity_controller" };
  ros::Publisher publisher;
  std_msgs::Float64MultiArray cmd;
  cmd.data.resize(7, 0.0);
  while (ros::ok()) {  // exerimental
    for (int i = 0; i < robot_ns.size(); i++) {
      for (int j = 0; j < controller.size(); i++) {
        publisher = nh.advertise<std_msgs::Float64MultiArray>(robot_ns[i] + controller[j] + "/command", 1);
        publisher.publish(cmd);
      }
    }
    ros::shutdown();
  }
}
void CartController::resetFt() {
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>(robot_ns + "ft_sensor/reset_offset");
  std_srvs::Empty srv;
  client.call(srv);
}

void CartController::initWithJnt(const KDL::JntArray& q_init) {
  // this->resetFt();
}

void CartController::getVelocity(const KDL::Frame& frame, const KDL::Frame& prev_frame, const double& dt, KDL::Twist& twist) const {
  Eigen::Affine3d T, T_prev;
  VectorXd vel(6);
  tf::transformKDLToEigen(frame, T);
  tf::transformKDLToEigen(prev_frame, T_prev);

  vel.head(3) = (T.translation() - T_prev.translation()) / dt;
  vel.tail(3) = rotation_util::getQuaternionError(Quaterniond(T.rotation()), Quaterniond(T_prev.rotation())) / dt;

  // std::cout << vel.transpose() << std::endl;

  tf::twistEigenToKDL(vel, twist);
  // vel.tail(3) = rotation_util::getQuaternionError(Quaterniond(T.rotation()), Quaterniond(T_prev.rotation())) / dt;
}

void CartController::cbJntState(const sensor_msgs::JointState::ConstPtr& msg) {
  KDL::JntArray q_cur(chain.getNrOfJoints());
  KDL::JntArray dq_cur(chain.getNrOfJoints());

  // static bool s_cbJntState.isFirst = true;
  // static bool s_cbJntState.initialized = false;

  unsigned int j = 0;
  for (size_t i = 0; i < chain_segs.size(); i++) {
    auto result = std::find(msg->name.begin(), msg->name.end(), chain_segs[i].getJoint().getName());

    if (result == msg->name.end())
      continue;

    q_cur.data[j] = msg->position[std::distance(msg->name.begin(), result)];
    dq_cur.data[j] = msg->velocity[std::distance(msg->name.begin(), result)];
    j++;
  }

  if (j != nJnt)
    return;

  if (s_cbJntState.isFirst) {
    if (!s_cbJntState.initialized) {
      s_cbJntState.initialized = moveInitPos(q_cur);
      // initialiuzed = moveInitPos();
      return;
    }

    initDesWithJnt(q_cur);
    initWithJnt(q_cur);
    s_cbJntState.isFirst = false;
    return;
  }

  std::lock_guard<std::mutex> lock(mtx);
  _q_cur = q_cur;
  _dq_cur = dq_cur;
  if (!flagJntState)
    flagJntState = true;
}

void CartController::cbArmMarker(const visualization_msgs::MarkerArray::ConstPtr& msg) {
  int nMarker = msg->markers.size();

  for (int i = 0; i < nMarker; i++) {
    if (msg->markers[i].id == ArmMarker::ArmMarkerID::ManipulabilityEllipsoid) {
      Quaterniond q;
      tf2::fromMsg(msg->markers[i].pose.orientation, q);

      std::lock_guard<std::mutex> lock(mtx);
      _userManipU = q.matrix();
      flagArmMarker = true;
    }
  }
}

// void CartController::cbForce(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
//   std::lock_guard<std::mutex> lock(mtx);
//   _force.header.stamp = msg->header.stamp;
//   _force.header.frame_id = robot_ns + chain_end;
//   _force.wrench = geometry_msgs_utility::transformFT(msg->wrench, Tft_eff.inverse());

//   // if (!flagEffPose)
//   //   flagEffPose = true;
// }

void CartController::initDesWithJnt(const KDL::JntArray& q_cur) {
  std::lock_guard<std::mutex> lock(mtx);
  fk_solver_ptr->JntToCart(q_cur, this->_des_eff_pose);
  this->_des_eff_vel = KDL::Twist::Zero();
}

int CartController::moveInitPos(const KDL::JntArray& q_cur) {
  ROS_INFO_STREAM_ONCE("Moving initial posiiton");
  // static bool isFirst = true;

  // static KDL::JntArray s_moveInitPos.q_des;
  // static KDL::JntArray s_moveInitPos.q_init = q_cur;

  if (s_moveInitPos.isFirst) {
    s_moveInitPos.q_init = q_cur;
    KDL::Frame init_eff_pose;

    tf::transformEigenToKDL(T_init, init_eff_pose);

    KDL::JntArray q_init_expect;
    q_init_expect.resize(nJnt);
    q_init_expect.data << 0.0, 0.5, 0.0, 1.0, 0.0, 1.6, 0.0;  // TODO: included in null space operation

    int rc;
    if (solver == SolverType::Trac_IK)
      rc = tracik_solver_ptr->CartToJnt(q_init_expect, init_eff_pose, s_moveInitPos.q_des);
    else if (solver == SolverType::KDL)
      rc = kdl_solver_ptr->CartToJnt(q_init_expect, init_eff_pose, s_moveInitPos.q_des);
    else if (solver == SolverType::MyIK)
      rc = myik_solver_ptr->CartToJnt(q_init_expect, init_eff_pose, s_moveInitPos.q_des);

    if (rc < 0)
      return false;

    s_moveInitPos.isFirst = false;

    s_moveInitPos.t_s = ros::Time::now();
  }

  const double T = 10.0;
  // static ros::Time s_moveInitPos.t_s = ros::Time::now();

  bool lastLoop = false;
  double s = 0.0, s2, s3, s4, s5;
  s = (ros::Time::now() - s_moveInitPos.t_s).toSec() / T;
  if (s > 1.0) {
    s = 1.0;
    lastLoop = true;
  }
  s2 = s * s;
  s3 = s2 * s;
  s4 = s3 * s;
  s5 = s4 * s;

  // min jerk trajectory
  VectorXd q_des_t = s_moveInitPos.q_init.data + (s_moveInitPos.q_des.data - s_moveInitPos.q_init.data) * (6.0 * s5 - 15.0 * s4 + 10.0 * s3);
  VectorXd dq_des_t = (s_moveInitPos.q_des.data - s_moveInitPos.q_init.data) * (30.0 * s4 - 60.0 * s3 + 30.0 * s2) / T;

  std_msgs::Float64MultiArray cmd;
  if (controller == ControllerType::Position) {
    cmd.data = std::vector<double>(q_des_t.data(), q_des_t.data() + q_des_t.rows() * q_des_t.cols());
    jntPosCmdPublisher.publish(cmd);
  } else if (controller == ControllerType::Velocity) {
    VectorXd dq_des_t_fb = dq_des_t + (1 - lastLoop) * 2.0 * (q_des_t - q_cur.data);
    cmd.data = std::vector<double>(dq_des_t_fb.data(), dq_des_t_fb.data() + dq_des_t_fb.rows() * dq_des_t_fb.cols());
    jntVelCmdPublisher.publish(cmd);
  } else if (controller == ControllerType::Torque) {
    // torque controller
  }

  if (!lastLoop)
    return false;

  return true;
}

void CartController::getDesEffPoseVel(const double& dt, const KDL::JntArray& q_cur, const KDL::JntArray& dq_cur, KDL::Frame& des_eff_pose, KDL::Twist& des_eff_vel) {
  bool disable;
  {
    std::lock_guard<std::mutex> lock(mtx);
    des_eff_pose = this->_des_eff_pose;
    des_eff_vel = this->_des_eff_vel;
    disable = this->_disable;
  }

  KDL::Frame frame;
  myik_solver_ptr->JntToCart(q_cur, frame);
  Affine3d T, Td;
  tf::transformKDLToEigen(frame, T);
  tf::transformKDLToEigen(des_eff_pose, Td);

  static ros::Time t0 = ros::Time::now();
  if (disable)
    t0 = ros::Time::now();

  double s = (ros::Time::now() - t0).toSec() / 2.0;
  if (s > 1.0)
    s = 1.0;  // 0-1

  Td.translation() = s * (Td.translation() - T.translation()) + T.translation();
  Td.linear() = Quaterniond(T.rotation()).slerp(s, Quaterniond(Td.rotation())).toRotationMatrix();

  tf::transformEigenToKDL(Td, des_eff_pose);

  KDL::Twist twist;
  // myik_solver_ptr->JntVelToCartVel(q_cur, dq_cur, twist);
  Matrix<double, 6, 1> v, vd;
  tf::twistKDLToEigen(twist, v);
  tf::twistKDLToEigen(des_eff_vel, vd);

  vd = s * (vd - v) + v;
  tf::twistEigenToKDL(vd, des_eff_vel);
}

void CartController::filterDesEffPoseVel(KDL::Frame& des_eff_pose, KDL::Twist& des_eff_vel) {
  Matrix<double, 6, 1> vel;
  tf::twistKDLToEigen(des_eff_vel, vel);
  for (int i = 0; i < 6; i++)
    vel(i) = velFilter[i].filter(vel(i));

  tf::twistEigenToKDL(vel, des_eff_vel);
}

void CartController::publishDesEffPoseVel(const KDL::Frame& des_eff_pose, const KDL::Twist& des_eff_vel) {
  ohrc_control::StateStamped state;
  state.header.stamp = ros::Time::now();
  state.state.pose = tf2::toMsg(des_eff_pose);
  state.state.twist.linear.x = des_eff_vel.vel[0];
  state.state.twist.linear.y = des_eff_vel.vel[1];
  state.state.twist.linear.z = des_eff_vel.vel[2];
  state.state.twist.angular.x = des_eff_vel.rot[0];
  state.state.twist.angular.y = des_eff_vel.rot[1];
  state.state.twist.angular.z = des_eff_vel.rot[2];
  desStatePublisher.publish(state);

  geometry_msgs::TransformStamped transform;
  // if ((ros::Time::now() - transform.header.stamp).toSec() < 1.0 / 50.0)
  //   return;

  transform = tf2::kdlToTransform(des_eff_pose);
  transform.header.stamp = ros::Time::now();

  transform.header.frame_id = robot_ns + chain_start;
  transform.child_frame_id = robot_ns + chain_end + "_d";
  br.sendTransform(transform);
}

void CartController::publishMarker(const KDL::JntArray q_cur) {
  visualization_msgs::Marker manipuMarker = myik_solver_ptr->getManipulabilityMarker(q_cur);
  manipuMarker.header.frame_id = robot_ns + chain_start;

  visualization_msgs::MarkerArray markers;
  markers.markers.push_back(manipuMarker);

  markerPublisher.publish(markers);
}

/**
 * \brief Starts controller
 * \param time Current time
 */
void CartController::starting(const ros::Time& time) {
  // check Gazebo is ready
  if (!gazebo_utility::checkGazeboInit())
    return;

  // start to subscribe topics
  spinner.start();

  // wait for subscribing registered topics
  subscriber_utility::checkSubTopic(subFlagPtrs, &mtx, robot_ns);

  this->resetFt();
  // TODO: reset ft sensor offset
  //
}

/**
 * \brief Stops controller
 * \param time Current time
 */
void CartController::stopping(const ros::Time& /*time*/) {
  std_msgs::Float64MultiArray cmd;
  cmd.data.resize(7, 0.0);
  for (int i = 0; i < nJnt; i++)
    jntVelCmdPublisher.publish(cmd);
}

void CartController::getIKInput(double dt, KDL::JntArray& q_cur, KDL::Frame& des_eff_pose, KDL::Twist& des_eff_vel) {
  KDL::JntArray dq_cur;
  {
    std::lock_guard<std::mutex> lock(mtx);
    q_cur = this->_q_cur;
    dq_cur = this->_dq_cur;
    // userManipU = this->_userManipU;
  }

  // userManipU << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  getDesEffPoseVel(dt, q_cur, dq_cur, des_eff_pose, des_eff_vel);
  // filterDesEffPoseVel(des_eff_pose, des_eff_vel);
  // publishDesEffPoseVel(des_eff_pose, des_eff_vel);
  // publishMarker(q_cur);
  //
}

void CartController::getState(KDL::JntArray& q_cur, KDL::JntArray& dq_cur) {
  {
    std::lock_guard<std::mutex> lock(mtx);
    q_cur = this->_q_cur;
    dq_cur = this->_dq_cur;
  }
}

/**
 * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
 * \param time   Current time
 * \param period Time since the last called to update
 */
void CartController::update() {
  update(ros::Time::now(), ros::Duration(1.0 / freq));
}
void CartController::update(const ros::Time& time, const ros::Duration& period) {
  // std::cout << robot_ns << std::endl;
  // // static ros::Rate r(freq);
  // static KDL::Frame des_eff_pose, current_eff_pose;
  // static KDL::Twist des_eff_vel;
  // static KDL::JntArray q_cur, dq_cur;
  // static std_msgs::Float64MultiArray cmd;
  // static Matrix3d userManipU;
  // static int rc;

  double dt = period.toSec();

  ROS_INFO_STREAM_ONCE("Start teleoperation");
  // while (ros::ok()) {
  {
    std::lock_guard<std::mutex> lock(mtx);
    //   q_cur = this->_q_cur;
    //   dq_cur = this->_dq_cur;
    userManipU = this->_userManipU;
  }

  userManipU << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  // getDesEffPoseVel(dt, q_cur, dq_cur, des_eff_pose, des_eff_vel);
  // // filterDesEffPoseVel(des_eff_pose, des_eff_vel);
  // publishDesEffPoseVel(des_eff_pose, des_eff_vel);
  // publishMarker(q_cur);
  getIKInput(dt, q_cur, des_eff_pose, des_eff_vel);

  if (controller == ControllerType::Position) {
    KDL::JntArray q_des(nJnt);
    static KDL::JntArray q_init = q_cur;

    if (solver == SolverType::Trac_IK)
      rc = tracik_solver_ptr->CartToJnt(q_init, des_eff_pose, q_des);
    else if (solver == SolverType::KDL)
      rc = kdl_solver_ptr->CartToJnt(q_init, des_eff_pose, q_des);
    else if (solver == SolverType::MyIK)
      rc = myik_solver_ptr->CartToJnt(q_init, des_eff_pose, q_des, dt);

    if (rc < 0) {
      ROS_WARN_STREAM("Failed to solve IK. Skip this control loop");
      return;
    }

    cmd.data = std::vector<double>(q_des.data.data(), q_des.data.data() + q_des.data.rows() * q_des.data.cols());
    jntPosCmdPublisher.publish(cmd);
    // ROS_INFO_STREAM(cmd);

    // q_init = q_des;
  } else if (controller == ControllerType::Velocity) {
    KDL::JntArray dq_des(nJnt);

    rc = myik_solver_ptr->CartToJntVel_qp(q_cur, des_eff_pose, des_eff_vel, dq_des, dt);
    // rc = myik_solver_ptr->CartToJntVel_qp_manipOpt(q_cur, des_eff_pose, des_eff_vel, dq_des, dt, userManipU);

    if (rc < 0) {
      ROS_WARN_STREAM("Failed to solve IK. Skip this control loop");
      return;
    }

    // low pass filter
    filterJnt(dq_des);

    cmd.data = std::vector<double>(dq_des.data.data(), dq_des.data.data() + dq_des.data.rows() * dq_des.data.cols());
    jntVelCmdPublisher.publish(cmd);
  } else if (controller == ControllerType::Torque) {
    // torque controller
  }

  // r.sleep();
  // }
}

void CartController::filterJnt(KDL::JntArray& q) {
  for (int i = 0; i < nJnt; i++)
    q.data(i) = jntFilter[i].filter(q.data(i));
}

int CartController::control() {
  // check Gazebo is ready
  if (!gazebo_utility::checkGazeboInit())
    return -1;

  // start to subscribe topics
  spinner.start();

  // wait for subscribing registered topics
  subscriber_utility::checkSubTopic(subFlagPtrs, &mtx);

  double dt = 1.0 / freq;
  // control
  ros::Rate r(freq);
  KDL::Frame des_eff_pose, current_eff_pose;
  KDL::Twist des_eff_vel;
  KDL::JntArray q_cur, dq_cur;
  std_msgs::Float64MultiArray cmd;
  Matrix3d userManipU;
  int rc;

  ROS_INFO_STREAM("Start teleoperation : " + robot_ns);
  while (ros::ok()) {
    {
      std::lock_guard<std::mutex> lock(mtx);
      q_cur = this->_q_cur;
      dq_cur = this->_dq_cur;
      userManipU = this->_userManipU;
    }

    userManipU << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    getDesEffPoseVel(dt, q_cur, dq_cur, des_eff_pose, des_eff_vel);
    // filterDesEffPoseVel(des_eff_pose, des_eff_vel);
    publishDesEffPoseVel(des_eff_pose, des_eff_vel);
    publishMarker(q_cur);

    if (controller == ControllerType::Position) {
      KDL::JntArray q_des(nJnt);
      static KDL::JntArray q_init = q_cur;

      if (solver == SolverType::Trac_IK)
        rc = tracik_solver_ptr->CartToJnt(q_init, des_eff_pose, q_des);
      else if (solver == SolverType::KDL)
        rc = kdl_solver_ptr->CartToJnt(q_init, des_eff_pose, q_des);
      else if (solver == SolverType::MyIK)
        rc = myik_solver_ptr->CartToJnt(q_init, des_eff_pose, q_des, dt);

      if (rc < 0) {
        ROS_WARN_STREAM("Failed to solve IK. Skip this control loop");
        continue;
      }

      cmd.data = std::vector<double>(q_des.data.data(), q_des.data.data() + q_des.data.rows() * q_des.data.cols());
      jntPosCmdPublisher.publish(cmd);
      // ROS_INFO_STREAM(cmd);

      q_init = q_des;
    } else if (controller == ControllerType::Velocity) {
      KDL::JntArray dq_des(nJnt);

      rc = myik_solver_ptr->CartToJntVel_qp(q_cur, des_eff_pose, des_eff_vel, dq_des, dt);
      // rc = myik_solver_ptr->CartToJntVel_qp_manipOpt(q_cur, des_eff_pose, des_eff_vel, dq_des, dt, userManipU);

      if (rc < 0) {
        ROS_WARN_STREAM("Failed to solve IK. Skip this control loop");
        continue;
      }

      // low pass filter
      filterJnt(dq_des);

      cmd.data = std::vector<double>(dq_des.data.data(), dq_des.data.data() + dq_des.data.rows() * dq_des.data.cols());
      jntVelCmdPublisher.publish(cmd);
    } else if (controller == ControllerType::Torque) {
      // torque controller
    }

    r.sleep();
  }

  return 1;
}
