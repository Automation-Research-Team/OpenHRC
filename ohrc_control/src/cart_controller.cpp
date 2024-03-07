#include "ohrc_control/cart_controller.hpp"

CartController::CartController(const std::string robot, const std::string hw_config, const std::string root_frame, const int index)
  : nh("~"), root_frame(root_frame), index(index) {
  init(robot, hw_config);
}

CartController::CartController(const std::string robot, const std::string root_frame, const int index) : nh("~"), root_frame(root_frame), index(index) {
  init(robot);
}

CartController::CartController(const std::string robot, const std::string root_frame) : nh("~"), root_frame(root_frame) {
  init(robot);
}

CartController::CartController() : nh("~") {
  std::string robot;
  nh.param("robot_ns", robot, std::string(""));
  root_frame = "world";

  init(robot);
}

void CartController::init(std::string robot) {
  this->init(robot, robot);
}

void CartController::init(std::string robot, std::string hw_config) {
  // nh_.setCallbackQueue(&queue);
  // spinner_.reset(new ros::AsyncSpinner(1, &queue));
  spinner.reset(new ros::AsyncSpinner(0));

  // std::signal(SIGINT, CartController::signal_handler);

  ROS_INFO_STREAM("namespace: " + robot);
  ROS_INFO_STREAM("hw_config: " + hw_config);
  if (robot != "")
    robot_ns = robot + "/";
  urdf_param = "/" + robot_ns + "robot_description";

  if (hw_config != "")
    hw_config_ns = hw_config + "/";

  if (!getInitParam())
    ros::shutdown();

  dt = 1.0 / freq;

  this->T_base_root = trans.getTransform(root_frame, robot_ns + chain_start, ros::Time(0), ros::Duration(10.0));

  tracik_solver_ptr.reset(new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param, dt, eps));

  KDL::JntArray ll, ul;  // lower joint limits, upper joint limits
  bool valid = tracik_solver_ptr->getKDLLimits(ll, ul);

  fk_solver_ptr.reset(new KDL::ChainFkSolverPos_recursive(chain));

  vik_solver_ptr.reset(new KDL::ChainIkSolverVel_pinv(chain));
  kdl_solver_ptr.reset(new KDL::ChainIkSolverPos_NR_JL(chain, ll, ul, *fk_solver_ptr, *vik_solver_ptr, 1, eps));

  myik_solver_ptr = std::make_shared<MyIK::MyIK>(chain_start, chain_end, urdf_param, eps, T_base_root);
  valid = myik_solver_ptr->getKDLChain(chain);
  chain_segs = chain.segments;

  nJnt = chain.getNrOfJoints();
  _q_cur.resize(nJnt);

  nh.param("/" + hw_config_ns + "initIKAngle", _q_init_expect, std::vector<double>(nJnt, 0.0));

  jntStateSubscriber = nh.subscribe("/" + robot_ns + "joint_states", 2, &CartController::cbJntState, this, th);
  subFlagPtrs.push_back(&flagJntState);

  if (useManipOpt) {
    userArmMarker = nh.subscribe("/arm_marker", 1, &CartController::cbArmMarker, this, th);
    subFlagPtrs.push_back(&flagArmMarker);
  }

  ROS_INFO_STREAM("Looking for force/torque sensor TF.");
  if (trans.canTransform(robot_ns + chain_end, robot_ns + "ft_sensor_link", ros::Time(0), ros::Duration(1.0))) {
    this->Tft_eff = trans.getTransform(robot_ns + chain_end, robot_ns + "ft_sensor_link", ros::Time(0), ros::Duration(1.0));
    subForce = nh.subscribe<geometry_msgs::WrenchStamped>("/" + robot_ns + "ft_sensor/filtered", 2, &CartController::cbForce, this, th);
    // pubEefForce = nh.advertise<geometry_msgs::WrenchStamped>("/" + robot_ns + "eef_force", 2);
    subFlagPtrs.push_back(&flagForce);
  } else
    ROS_WARN_STREAM("force/torque sensor TF was not found.");
  client = nh.serviceClient<std_srvs::Empty>("/" + robot_ns + "ft_filter/reset_offset");

  if (publisher == PublisherType::Trajectory)
    jntCmdPublisher = nh.advertise<trajectory_msgs::JointTrajectory>("/" + robot_ns + publisherTopicName + "/command", 1);
  else if (publisher == PublisherType::TrajectoryAction)
    jntCmdPublisher = nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/" + robot_ns + publisherTopicName + "/follow_joint_trajectory/goal", 1);
  else
    jntCmdPublisher = nh.advertise<std_msgs::Float64MultiArray>("/" + robot_ns + publisherTopicName + "/command", 2);

  desStatePublisher = nh.advertise<ohrc_msgs::State>("/" + robot_ns + "state/desired", 1000);
  curStatePublisher = nh.advertise<ohrc_msgs::State>("/" + robot_ns + "state/current", 1000);
  markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/" + robot_ns + "markers", 1);

  if (robot_ns != "")
    service = nh.advertiseService("/" + robot_ns + "reset", &CartController::resetService, this);

  Affine3d T_init_base = getTransform_base(robot_ns + initPoseFrame);
  T_init = Translation3d(initPose[0], initPose[1], initPose[2]) *
           (AngleAxisd(initPose[3], Vector3d::UnitX()) * AngleAxisd(initPose[4], Vector3d::UnitY()) * AngleAxisd(initPose[5], Vector3d::UnitZ()));
  T_init = T_init_base * T_init;

  for (int i = 0; i < 6; i++)
    // velFilter.push_back(butterworth(2, 10.0, freq));
    velFilter.push_back(butterworth(2, freq / 3.0, freq));

  for (int i = 0; i < nJnt; i++)
    // jntFilter.push_back(butterworth(2, 20.0, freq));
    jntFilter.push_back(butterworth(2, freq / 5.0, freq));
}

void CartController::updateFilterCutoff(const double velFreq, const double jntFreq) {
  for (int i = 0; i < 6; i++)
    velFilter[i] = butterworth(2, velFreq, freq);
  // velFilter.push_back(butterworth(2, freq / 3.0, freq));

  for (int i = 0; i < nJnt; i++)
    jntFilter[i] = butterworth(2, jntFreq, freq);
  // jntFilter.push_back(butterworth(2, freq / 3.0, freq));
}

bool CartController::getInitParam() {
  nh.param("/" + hw_config_ns + "chain_start", chain_start, std::string(""));
  if (root_frame == "")
    root_frame = chain_start;

  nh.param("/" + hw_config_ns + "chain_end", chain_end, std::string(""));

  nh.param("useManipulabilityOpt", useManipOpt, false);

  // std::cout << "/" + hw_config_ns + "chain_start: " << chain_start << std::endl;
  if (chain_start == "" || chain_end == "") {
    ROS_FATAL("Missing chain info in launch file");
    // exit(-1);
    return false;
  }

  std::string solver_str;
  if (!nh.param("solver", solver_str, std::string("MyIK")))
    ROS_WARN_STREAM("Solver type is not choisen {Trac_Ik, KDL, MyIK}: Default MyIK");
  else
    ROS_INFO_STREAM("Solver: " << solver_str);

  solver = magic_enum::enum_cast<SolverType>(solver_str).value_or(SolverType::None);
  if (solver == SolverType::None) {
    ROS_FATAL("Solver type is not correctly choisen from {Trac_IK, KDL, MyIK}");
    return false;
  }

  std::string controller_str;
  if (!nh.param("controller", controller_str, std::string("Velocity")))
    ROS_WARN_STREAM("Controller is not choisen {Position, Velocity, Torque}: Default Velocity");
  else
    ROS_INFO_STREAM("Controller: " << controller_str);

  controller = magic_enum::enum_cast<ControllerType>(controller_str).value_or(ControllerType::None);
  if (controller == ControllerType::None) {
    ROS_FATAL("Controller type is not correctly choisen from {Position, Velocity, Torque}");
    return false;
  }

  std::string publisher_str;
  if (!nh.param("/" + hw_config_ns + "publisher", publisher_str, std::string("Velocity")))
    ROS_WARN_STREAM("Publisher is not choisen {Position, Velocity, Torque, Trajectory, TrajectoryAction}: Default Velocity");
  else
    ROS_INFO_STREAM("Publisher: " << publisher_str);

  publisher = magic_enum::enum_cast<PublisherType>(publisher_str).value_or(PublisherType::None);
  if (publisher == PublisherType::None) {
    ROS_FATAL("Publisher type is not correctly choisen from {Position, Velocity, Torque, Trajectory, TrajectoryAction}");
    return false;
  }

  std::string defaltPublisherTopicName;
  if (publisher == PublisherType::Position)
    defaltPublisherTopicName = "joint_position_controller";
  else if (publisher == PublisherType::Velocity)
    defaltPublisherTopicName = "joint_velocity_controller";
  else if (publisher == PublisherType::Trajectory || publisher == PublisherType::TrajectoryAction)
    defaltPublisherTopicName = "joint_trajectory_controller";

  if (!nh.param("/" + hw_config_ns + "topic_namespace", publisherTopicName, defaltPublisherTopicName)) {
    ROS_WARN_STREAM("Publisher is not configured: " << defaltPublisherTopicName);
  }

  if (!nh.getParam("control_freq", freq)) {
    ROS_FATAL_STREAM("Failed to get the control_freq of the robot system");
    return false;
  }

  double freq_bound;
  if (nh.getParam("/" + hw_config_ns + "freq_bound", freq_bound)) {
    ROS_WARN_STREAM("freq is bounded by " << freq_bound);
    freq = freq_bound;
  }

  // subFlagPtrs.push_back(&flagEffPose);
  // nh.param("initIKAngle", _q_init_expect, std::vector<double>(nJnt, 0.0));

  // f
  //  _q_init_expect[i] = 0.0;
  // if (_q_init_expect.size() != nJnt) {
  // ROS_ERROR_STREAM("Size of initIKAngle should be equal to number of joints");
  // return false;
  // }
  // std::cout << init_q_expect << std::endl;

  nh.param("/" + hw_config_ns + "initial_pose_frame", initPoseFrame, chain_start);
  nh.param("/" + hw_config_ns + "initial_pose", initPose, std::vector<double>{ 0.45, 0.0, 0.85, 0.0, M_PI, -M_PI_2 });

  return true;
}

CartController::~CartController() {
}

bool CartController::resetService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  resetPose();
  return true;
}

void CartController::resetPose() {
  ROS_WARN_STREAM("The robot will moves to the initail pose!");

  s_cbJntState.isFirst = true;
  initialized = false;
  s_moveInitPos.isFirst = true;
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
  ROS_INFO_STREAM("Called reset ft service.");

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
  KDL::JntArray q_cur(nJnt);
  KDL::JntArray dq_cur(nJnt);

  bool allJntFound = false;

  unsigned int j = 0;
  std::vector<std::string> nameJnt(nJnt);
  std::vector<int> idxSegJnt(nJnt);
  for (size_t i = 0; i < chain_segs.size(); i++) {
    auto result = std::find(msg->name.begin(), msg->name.end(), chain_segs[i].getJoint().getName());

    if (result == msg->name.end())
      continue;

    int idx = std::distance(msg->name.begin(), result);

    q_cur.data[j] = msg->position[idx];
    dq_cur.data[j] = msg->velocity[idx];
    nameJnt[j] = msg->name[idx];
    idxSegJnt[j] = i;
    j++;

    if (j == nJnt) {
      allJntFound = true;
      break;
    }
  }

  if (!allJntFound)
    return;

  if (s_cbJntState.isFirst) {
    if (!initialized) {
      initialized = moveInitPos(q_cur, nameJnt, idxSegJnt);
      return;
    }

    initDesWithJnt(q_cur);
    initWithJnt(q_cur);
    s_cbJntState.isFirst = false;
    return;
  }

  std::lock_guard<std::mutex> lock(mtx_q);

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

void CartController::cbForce(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx);
  _force.header.stamp = msg->header.stamp;
  _force.header.frame_id = robot_ns + chain_end;
  _force.wrench = geometry_msgs_utility::transformFT(msg->wrench, Tft_eff);

  // this->pubEefForce.publish(_force);

  if (!flagForce)
    flagForce = true;
}

void CartController::initDesWithJnt(const KDL::JntArray& q_cur) {
  std::lock_guard<std::mutex> lock(mtx);
  fk_solver_ptr->JntToCart(q_cur, this->_des_eef_pose);
  this->_des_eef_vel = KDL::Twist::Zero();
}

int CartController::moveInitPos(const KDL::JntArray& q_cur, const std::vector<std::string> nameJnt, std::vector<int> idxSegJnt) {
  ROS_INFO_STREAM_ONCE("Moving initial posiiton");

  if (s_moveInitPos.isFirst) {
    this->nameJnt = nameJnt;
    s_moveInitPos.q_initial = q_cur;
    KDL::Frame init_eef_pose;

    tf::transformEigenToKDL(T_init, init_eef_pose);

    KDL::JntArray q_init_expect;
    q_init_expect.resize(nJnt);
    q_init_expect.data = VectorXd::Map(&_q_init_expect[0], nJnt);

    int rc;
    switch (solver) {
      case SolverType::Trac_IK:
        rc = tracik_solver_ptr->CartToJnt(q_init_expect, init_eef_pose, s_moveInitPos.q_des);
        break;

      case SolverType::KDL:
        rc = kdl_solver_ptr->CartToJnt(q_init_expect, init_eef_pose, s_moveInitPos.q_des);
        break;

      case SolverType::MyIK:
        myik_solver_ptr->setNameJnt(nameJnt);
        myik_solver_ptr->setIdxSegJnt(idxSegJnt);
        rc = myik_solver_ptr->CartToJnt(q_init_expect, init_eef_pose, s_moveInitPos.q_des, 5.0);
        break;
    }

    if (rc < 0) {
      ROS_ERROR_STREAM("Failed to find initial joint angle. Please check if the initial position is appropriate.");
      return false;
    } else
      ROS_INFO_STREAM("Successfuly soloved initial joint angles.");

    s_moveInitPos.isFirst = false;

    s_moveInitPos.t_s = ros::Time::now();

    this->q_rest = s_moveInitPos.q_des;
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
  VectorXd q_des_t = s_moveInitPos.q_initial.data + (s_moveInitPos.q_des.data - s_moveInitPos.q_initial.data) * (6.0 * s5 - 15.0 * s4 + 10.0 * s3);
  VectorXd dq_des_t = (s_moveInitPos.q_des.data - s_moveInitPos.q_initial.data) * (30.0 * s4 - 60.0 * s3 + 30.0 * s2) / T;

  switch (publisher) {
    case PublisherType::Position:
      sendPositionCmd(q_des_t);
      break;
    case PublisherType::Velocity:
      sendVelocityCmd(q_des_t, dq_des_t, q_cur, lastLoop);
      break;
    case PublisherType::Torque:
      ROS_ERROR_STREAM_ONCE("Torque controller is still not implemented...");
      break;
    case PublisherType::Trajectory:
      sendTrajectoryCmd(s_moveInitPos.q_des.data, T * (1.0 - s));
      break;
    case PublisherType::TrajectoryAction:
      sendTrajectoryActionCmd(s_moveInitPos.q_des.data, T * (1.0 - s));
      ros::Duration(T).sleep();
      lastLoop = true;
      break;
    default:
      ROS_ERROR_STREAM_ONCE("This publisher is not implemented...");
      break;
  }

  if (!lastLoop)
    return false;

  return true;
}

void CartController::sendPositionCmd(const VectorXd& q_des) {
  std_msgs::Float64MultiArray cmd;
  cmd.data = std::vector<double>(q_des.data(), q_des.data() + q_des.rows() * q_des.cols());
  jntCmdPublisher.publish(cmd);
}

void CartController::sendVelocityCmd(const VectorXd& dq_des) {
  sendPositionCmd(dq_des);
}

void CartController::sendVelocityCmd(const VectorXd& q_des, const VectorXd& dq_des, const KDL::JntArray& q_cur, const bool& lastLoop) {
  double kp = 4.0;  // feedback p gain
  std_msgs::Float64MultiArray cmd;
  VectorXd dq_des_ = dq_des + (1.0 - (double)lastLoop) * kp * (q_des - q_cur.data);
  sendVelocityCmd(dq_des_);
}

void CartController::getTrajectoryCmd(const VectorXd& q_des, const double& T, trajectory_msgs::JointTrajectory& cmd_trj) {
  cmd_trj.header.stamp = ros::Time::now();
  cmd_trj.points.resize(1);
  cmd_trj.points[0].time_from_start = ros::Duration(T);

  for (int i = 0; i < nJnt; i++) {
    cmd_trj.joint_names.push_back(nameJnt[i]);
    cmd_trj.points[0].positions.push_back(q_des[i]);
    cmd_trj.points[0].velocities.push_back(0.0);
    cmd_trj.points[0].accelerations.push_back(0.0);
  }
}

void CartController::getTrajectoryCmd(const VectorXd& q_des, const VectorXd& dq_des, const double& T, trajectory_msgs::JointTrajectory& cmd_trj) {
  // cmd_trj.header.stamp = ros::Time::now();
  cmd_trj.points.resize(1);
  cmd_trj.points[0].time_from_start = ros::Duration(T);

  for (int i = 0; i < nJnt; i++) {
    cmd_trj.joint_names.push_back(nameJnt[i]);
    cmd_trj.points[0].positions.push_back(q_des[i]);
    cmd_trj.points[0].velocities.push_back(dq_des[i]);
    cmd_trj.points[0].accelerations.push_back(0.0);
  }
}
void CartController::sendTrajectoryCmd(const VectorXd& q_des, const double& T) {
  trajectory_msgs::JointTrajectory cmd_trj;
  getTrajectoryCmd(q_des, T, cmd_trj);
  jntCmdPublisher.publish(cmd_trj);
}

void CartController::sendTrajectoryCmd(const VectorXd& q_des, const VectorXd& dq_des, const double& T) {
  trajectory_msgs::JointTrajectory cmd_trj;
  getTrajectoryCmd(q_des, dq_des, T, cmd_trj);
  jntCmdPublisher.publish(cmd_trj);
}

void CartController::sendTrajectoryActionCmd(const VectorXd& q_des, const double& T) {
  control_msgs::FollowJointTrajectoryActionGoal cmd_trjAction;
  getTrajectoryCmd(q_des, T, cmd_trjAction.goal.trajectory);
  jntCmdPublisher.publish(cmd_trjAction);
}
void CartController::sendTrajectoryActionCmd(const VectorXd& q_des, const VectorXd& dq_des, const double& T) {
  control_msgs::FollowJointTrajectoryActionGoal cmd_trjAction;
  getTrajectoryCmd(q_des, dq_des, T, cmd_trjAction.goal.trajectory);
  cmd_trjAction.header.stamp = cmd_trjAction.goal.trajectory.header.stamp;
  jntCmdPublisher.publish(cmd_trjAction);
}
#if 0
void CartController::getDesEffPoseVel(const double& dt, const KDL::JntArray& q_cur, const KDL::JntArray& dq_cur, KDL::Frame& des_eef_pose, KDL::Twist& des_eef_vel) {
  bool disable;
  {
    std::lock_guard<std::mutex> lock(mtx);
    des_eef_pose = this->_des_eef_pose;
    des_eef_vel = this->_des_eef_vel;
    disable = this->_disable;
  }

  KDL::Frame frame;
  myik_solver_ptr->JntToCart(q_cur, frame);
  Affine3d T, Td;
  tf::transformKDLToEigen(frame, T);
  tf::transformKDLToEigen(des_eef_pose, Td);

  static ros::Time t0 = ros::Time::now();
  if (disable)
    t0 = ros::Time::now();

  double s = (ros::Time::now() - t0).toSec() / 2.0;
  if (s > 1.0)
    s = 1.0;  // 0-1

  Td.translation() = s * (Td.translation() - T.translation()) + T.translation();
  Td.linear() = Quaterniond(T.rotation()).slerp(s, Quaterniond(Td.rotation())).toRotationMatrix();

  tf::transformEigenToKDL(Td, des_eef_pose);

  KDL::Twist twist;
  // myik_solver_ptr->JntVelToCartVel(q_cur, dq_cur, twist);
  Matrix<double, 6, 1> v, vd;
  tf::twistKDLToEigen(twist, v);
  tf::twistKDLToEigen(des_eef_vel, vd);

  vd = s * (vd - v) + v;
  tf::twistEigenToKDL(vd, des_eef_vel);
}
#endif
void CartController::filterDesEffPoseVel(KDL::Frame& des_eef_pose, KDL::Twist& des_eef_vel) {
  Matrix<double, 6, 1> vel;
  tf::twistKDLToEigen(des_eef_vel, vel);
  for (int i = 0; i < 6; i++)
    vel(i) = velFilter[i].filter(vel(i));

  tf::twistEigenToKDL(vel, des_eef_vel);
}

void CartController::publishState(const KDL::Frame& pose, const KDL::Twist& vel, ros::Publisher* publisher) {
  this->publishState(pose, vel, geometry_msgs::Wrench(), publisher);
}

void CartController::publishState(const KDL::Frame& pose, const KDL::Twist& vel, const geometry_msgs::Wrench& wrench, ros::Publisher* publisher) {
  ohrc_msgs::State state;
  state.header.stamp = ros::Time::now();
  state.pose = tf2::toMsg(pose);
  state.twist.linear.x = vel.vel[0];
  state.twist.linear.y = vel.vel[1];
  state.twist.linear.z = vel.vel[2];
  state.twist.angular.x = vel.rot[0];
  state.twist.angular.y = vel.rot[1];
  state.twist.angular.z = vel.rot[2];

  state.enabled = this->getOperationEnable();

  state.wrench = wrench;
  publisher->publish(state);
}

void CartController::publishDesEffPoseVel(const KDL::Frame& des_eef_pose, const KDL::Twist& des_eef_vel) {
  publishState(des_eef_pose, des_eef_vel, &desStatePublisher);
}

void CartController::publishCurEffPoseVel(const KDL::Frame& cur_eef_pose, const KDL::Twist& cur_eef_vel) {
  std::lock_guard<std::mutex> lock(mtx);
  publishState(cur_eef_pose, cur_eef_vel, _force.wrench, &curStatePublisher);
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
  // spinner_->start();
  spinner->start();

  // wait for subscribing registered topics
  subscriber_utility::checkSubTopic(subFlagPtrs, &mtx, robot_ns);

  this->resetFt();
  // TODO: reset ft sensor offset
  //
  updateCurState();
}

/**
 * \brief Stops controller
 * \param time Current time
 */
void CartController::stopping(const ros::Time& /*time*/) {
  if (controller == ControllerType::Velocity) {
    std_msgs::Float64MultiArray cmd;
    cmd.data.resize(nJnt, 0.0);
    jntCmdPublisher.publish(cmd);
  }
}

void CartController::getIKInput(double dt, KDL::JntArray& q_cur, KDL::Frame& des_eef_pose, KDL::Twist& des_eef_vel) {
  KDL::JntArray dq_cur;
  KDL::Frame cur_eef_pose;
  KDL::Twist cur_eef_vel;

  // getState(q_cur, dq_cur);

  // // userManipU << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  // getDesEffPoseVel(dt, q_cur, dq_cur, des_eef_pose, des_eef_vel);
  // filterDesEffPoseVel(des_eef_pose, des_eef_vel);

  getState(q_cur, dq_cur, cur_eef_pose, cur_eef_vel);
  getDesState(cur_eef_pose, cur_eef_vel, des_eef_pose, des_eef_vel);
  publishDesEffPoseVel(des_eef_pose, des_eef_vel);
  publishCurEffPoseVel(cur_eef_pose, cur_eef_vel);
  // publishMarker(q_cur);
  //
}

// void CartController::getState(KDL::JntArray& q_cur, KDL::JntArray& dq_cur) {
//   {
//     std::lock_guard<std::mutex> lock(mtx);
//     q_cur = this->_q_cur;
//     dq_cur = this->_dq_cur;
//   }
// }

void CartController::getDesState(const KDL::Frame& cur_pose, const KDL::Twist& cur_vel, KDL::Frame& des_pose, KDL::Twist& des_vel) {
  bool disable, passThrough;
  {
    std::lock_guard<std::mutex> lock(mtx);
    des_pose = this->_des_eef_pose;
    des_vel = this->_des_eef_vel;
    disable = this->_disable;
    passThrough = this->_passThrough;
  }

  Affine3d T, Td;
  tf::transformKDLToEigen(cur_pose, T);
  tf::transformKDLToEigen(des_pose, Td);

  static ros::Time t0 = ros::Time::now();
  if (disable) {
    t0 = ros::Time::now();
    disablePoseFeedback();
  } else
    enablePoseFeedback();

  double s = (ros::Time::now() - t0).toSec() / 3.0;
  if (s > 1.0 || passThrough)
    s = 1.0;  // 0-1

  Td.translation() = s * (Td.translation() - T.translation()) + T.translation();
  Td.linear() = Quaterniond(T.rotation()).slerp(s, Quaterniond(Td.rotation())).toRotationMatrix();

  tf::transformEigenToKDL(Td, des_pose);

  // KDL::Twist twist;
  Matrix<double, 6, 1> v, vd;
  v << 0, 0, 0, 0, 0, 0;
  // tf::twistKDLToEigen(cur_vel, v);
  tf::twistKDLToEigen(des_vel, vd);

  vd = s * (vd - v) + v;
  tf::twistEigenToKDL(vd, des_vel);
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
  if ((time - prev_time) < period)
    return;

  prev_time = time;

  updateCurState();
  // std::cout << robot_ns << std::endl;
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

  // getDesEffPoseVel(dt, q_cur, dq_cur, des_eef_pose, des_eef_vel);
  // // filterDesEffPoseVel(des_eef_pose, des_eef_vel);
  // publishDesEffPoseVel(des_eef_pose, des_eef_vel);
  // publishMarker(q_cur);
  getIKInput(dt, q_cur, des_eef_pose, des_eef_vel);

  static KDL::JntArray dq_des(nJnt);
  static KDL::JntArray q_des(q_cur);  // TODO: this might should be initialized with q_init

  if (controller == ControllerType::Position) {
    // KDL::JntArray q_des(nJnt);
    // static KDL::JntArray q_init = q_cur;
    KDL::JntArray q_des_prev = q_des;

    switch (solver) {
      case SolverType::Trac_IK:
        rc = tracik_solver_ptr->CartToJnt(q_cur, des_eef_pose, q_des);
        break;
      case SolverType::KDL:
        rc = kdl_solver_ptr->CartToJnt(q_cur, des_eef_pose, q_des);
        break;
      case SolverType::MyIK:
        rc = myik_solver_ptr->CartToJnt(q_cur, des_eef_pose, q_des, dt);
        break;
    }
    if (rc < 0) {
      ROS_WARN_STREAM("Failed to solve IK. Skip this control loop");
      return;
    }

    // low pass filter
    filterJnt(q_des);

    dq_des.data = (q_des.data - q_des_prev.data) / dt;

    sendPosCmd(q_des, dq_des, dt);

    q_des_prev = q_des;

  } else if (controller == ControllerType::Velocity) {
    rc = myik_solver_ptr->CartToJntVel_qp(q_cur, des_eef_pose, des_eef_vel, dq_des, dt);

    if (rc < 0) {
      ROS_WARN_STREAM("Failed to solve IK. Skip this control loop");
      return;
    }

    // low pass filter
    filterJnt(dq_des);

    q_des.data += dq_des.data * dt;

    sendVelCmd(q_des, dq_des, dt);

  } else if (controller == ControllerType::Torque) {
    // torque controller
  } else
    ROS_ERROR_STREAM("Not Implemented!");
}

void CartController::sendPosCmd(const KDL::JntArray& q_des, const KDL::JntArray& dq_des, const double& dt) {
  switch (publisher) {
    case PublisherType::Position:
      sendPositionCmd(q_des.data);
      break;
    case PublisherType::Velocity:
      sendVelocityCmd(dq_des.data);
      break;
    case PublisherType::Trajectory:
      sendTrajectoryCmd(q_des.data, dt);
      break;
    case PublisherType::TrajectoryAction:
      sendTrajectoryActionCmd(q_des.data, dt);
      break;
    default:
      ROS_ERROR_STREAM_ONCE("This controller is not implemented...");
      break;
  }
}

void CartController::sendVelCmd(const KDL::JntArray& q_des, const KDL::JntArray& dq_des, const double& dt) {
  switch (publisher) {
    case PublisherType::Position:
      sendPositionCmd(q_des.data);
      break;
    case PublisherType::Velocity:
      sendVelocityCmd(dq_des.data);
      break;
    case PublisherType::Trajectory:
      sendTrajectoryCmd(q_des.data, dq_des.data, dt);
      break;
    case PublisherType::TrajectoryAction:
      sendTrajectoryActionCmd(q_des.data, dq_des.data, dt);
      break;
    default:
      ROS_ERROR_STREAM_ONCE("This controller is not implemented...");
      break;
  }
}

void CartController::filterJnt(KDL::JntArray& q) {
  for (int i = 0; i < nJnt; i++)
    q.data(i) = jntFilter[i].filter(q.data(i));
}

int CartController::control() {
  starting(ros::Time::now());
  ros::Duration(3.0).sleep();

  while (ros::ok())
    update(ros::Time::now(), ros::Duration(1.0 / freq));

  stopping(ros::Time::now());

  return 1;
}

#if 0
int CartController::control() {
  // check Gazebo is ready
  if (!gazebo_utility::checkGazeboInit())
    return -1;

  // start to subscribe topics
  spinner_->start();
  spinner->start();

  // wait for subscribing registered topics
  subscriber_utility::checkSubTopic(subFlagPtrs, &mtx);

  double dt = 1.0 / freq;
  // control
  ros::Rate r(freq);
  KDL::Frame des_eef_pose, current_eef_pose;
  KDL::Twist des_eef_vel;
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

    getDesEffPoseVel(dt, q_cur, dq_cur, des_eef_pose, des_eef_vel);
    // filterDesEffPoseVel(des_eef_pose, des_eef_vel);
    // publishDesEffPoseVel(des_eef_pose, des_eef_vel);
    publishMarker(q_cur);

    if (controller == ControllerType::Position) {
      KDL::JntArray q_des(nJnt);
      static KDL::JntArray q_init = q_cur;

      if (solver == SolverType::Trac_IK)
        rc = tracik_solver_ptr->CartToJnt(q_init, des_eef_pose, q_des);
      else if (solver == SolverType::KDL)
        rc = kdl_solver_ptr->CartToJnt(q_init, des_eef_pose, q_des);
      else if (solver == SolverType::MyIK)
        rc = myik_solver_ptr->CartToJnt(q_init, des_eef_pose, q_des, dt);

      if (rc < 0) {
        ROS_WARN_STREAM("Failed to solve IK. Skip this control loop");
        continue;
      }

      cmd.data = std::vector<double>(q_des.data.data(), q_des.data.data() + q_des.data.rows() * q_des.data.cols());
      jntCmdPublisher.publish(cmd);
      // ROS_INFO_STREAM(cmd);

      q_init = q_des;
    } else if (controller == ControllerType::Velocity) {
      KDL::JntArray dq_des(nJnt);

      rc = myik_solver_ptr->CartToJntVel_qp(q_cur, des_eef_pose, des_eef_vel, dq_des, dt);
      // rc = myik_solver_ptr->CartToJntVel_qp_manipOpt(q_cur, des_eef_pose, des_eef_vel, dq_des, dt, userManipU);

      if (rc < 0) {
        ROS_WARN_STREAM("Failed to solve IK. Skip this control loop");
        continue;
      }

      // low pass filter
      filterJnt(dq_des);

      cmd.data = std::vector<double>(dq_des.data.data(), dq_des.data.data() + dq_des.data.rows() * dq_des.data.cols());
      jntCmdPublisher.publish(cmd);
    } else
      ROS_ERROR_STREAM("Not Implemented.");

    r.sleep();
  }

  return 1;
}
#endif