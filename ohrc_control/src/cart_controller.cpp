#include "ohrc_control/cart_controller.hpp"

CartController::CartController(rclcpp::Node::SharedPtr& node, const std::string robot, const std::string hw_config, const std::string root_frame, const int index,
                               const ControllerType controller, const double freq)
  : Node("cart_controller_" + robot), root_frame(root_frame), index(index), controller(controller), freq(freq) {
  node = std::shared_ptr<rclcpp::Node>(this);
  this->node = node;
  trans.reset(new TransformUtility(this->node));
  init(robot, hw_config);
}

CartController::CartController(rclcpp::Node::SharedPtr& node, const std::string robot, const std::string root_frame, const int index, const ControllerType controller,
                               const double freq)
  : Node("cart_controller_" + robot), root_frame(root_frame), index(index), controller(controller), freq(freq) {
  node = std::shared_ptr<rclcpp::Node>(this);
  this->node = node;
  trans.reset(new TransformUtility(this->node));
  init(robot);
}

CartController::CartController(rclcpp::Node::SharedPtr& node, const std::string robot, const std::string root_frame, const ControllerType controller, const double freq)
  : Node("cart_controller_" + robot), root_frame(root_frame), controller(controller), freq(freq) {
  node = std::shared_ptr<rclcpp::Node>(this);
  this->node = node;
  trans.reset(new TransformUtility(this->node));
  init(robot);
}

// CartController::CartController() : Node("ohrc_cart_controller_" + robot) {
//   std::string robot;
//   // nh.param("robot_ns", robot, std::string(""));
//   this->declare_parameter("robot_ns", "");
//   robot = this->get_parameter("robot_ns").as_string();
//   root_frame = "world";

//   init(robot);
// }

void CartController::init(std::string robot) {
  this->init(robot, robot);
}

void CartController::init(std::string robot, std::string hw_config) {
  // this->node = std::shared_ptr<rclcpp::Node>(this);
  // nh_.setCallbackQueue(&queue);
  // spinner_.reset(new ros::AsyncSpinner(1, &queue));
  // spinner.reset(new ros::AsyncSpinner(0));

  // std::signal(SIGINT, CartController::signal_handler);

  header = "[" + hw_config + "(ns: '" + robot + "')] ";
  RCLCPP_INFO_STREAM(node->get_logger(), "Initializing " + header);

  if (robot != "")
    robot_ns = robot + ".";
  urdf_param = robot_ns + "robot_description";

  if (hw_config != "")
    hw_config_ns = hw_config + ".";

  if (!getRosParams()) {
    RCLCPP_FATAL_STREAM(node->get_logger(), "Failed to get ROS parameters for" << header);
    rclcpp::shutdown();
  }

  initMembers();

  // tracik_solver_ptr.reset(new TRAC_IK::TRAC_IK(this->shared_from_this(), chain_start, chain_end, urdf_param, dt, eps));

  KDL::JntArray ll, ul;  // lower joint limits, upper joint limits
  // bool valid = tracik_solver_ptr->getKDLLimits(ll, ul);

  fk_solver_ptr.reset(new KDL::ChainFkSolverPos_recursive(chain));

  // vik_solver_ptr.reset(new KDL::ChainIkSolverVel_pinv(chain));
  // kdl_solver_ptr.reset(new KDL::ChainIkSolverPos_NR_JL(chain, ll, ul, *fk_solver_ptr, *vik_solver_ptr, 1, eps));

  // myik_solver_ptr = std::make_shared<MyIK::MyIK>(chain_start, chain_end, urdf_param, eps, T_base_root);
  // bool valid = myik_solver_ptr->getKDLChain(chain);
  // chain_segs = chain.segments;

  // nJnt = chain.getNrOfJoints();
  // _q_cur.resize(nJnt);

  // nh.param("/" + hw_config_ns + "initIKAngle", _q_init_expect, std::vector<double>(nJnt, 0.0));
  // this->declare_parameter("/" + hw_config_ns + "initIKAngle", std::vector<double>(nJnt, 0.0));
  // this->get_parameter("/" + hw_config_ns + "initIKAngle", _q_init_expect);

  // jntStateSubscriber = nh.subscribe("/" + robot_ns + "joint_states", 1, &CartController::cbJntState, this, th);
  std::string jnt_state_topic = "/" + robot_ns + "joint_states";
  subJntState = node->create_subscription<sensor_msgs::msg::JointState>(jnt_state_topic, rclcpp::QoS(1), std::bind(&CartController::cbJntState, this, _1));
  subFlagPtrs.push_back(&flagJntState);

  std::string ft_sensor_link, ft_topic;

  RclcppUtility::declare_and_get_parameter(this->node, robot_ns + "ft_sensor_link", std::string("ft_sensor_link"), ft_sensor_link);
  RclcppUtility::declare_and_get_parameter(this->node, robot_ns + "ft_topic", std::string("ft_sensor/filtered"), ft_topic);

  RCLCPP_INFO_STREAM(node->get_logger(), "Looking for force/torque sensor TF: " << ft_sensor_link << ", topic: " << ft_topic);
  if (trans->canTransform(robot_ns + chain_end, robot_ns + ft_sensor_link, rclcpp::Time(0), rclcpp::Duration(1.0, 0))) {
    this->Tft_eff = trans->getTransform(robot_ns + chain_end, robot_ns + ft_sensor_link, rclcpp::Time(0), rclcpp::Duration(1., 0));
    subForce = node->create_subscription<geometry_msgs::msg::WrenchStamped>(ft_topic, rclcpp::QoS(1), std::bind(&CartController::cbForce, this, _1));
    pubEefForce = node->create_publisher<geometry_msgs::msg::WrenchStamped>("/" + robot_ns + "eef_force", rclcpp::QoS(1));
    subFlagPtrs.push_back(&flagForce);
    this->ftFound = true;
  } else
    RCLCPP_WARN_STREAM(node->get_logger(), "force/torque sensor was not found. Compliance control does not work.");

  // client = nh.serviceClient<std_srvs::Empty>("/" + robot_ns + "ft_filter/reset_offset");
  client = node->create_client<std_srvs::srv::Empty>("/" + robot_ns + "ft_filter/reset_offset");

  // if (publisher == PublisherType::Trajectory){
  //   // jntCmdPublisher = nh.advertise<trajectory_msgs::JointTrajectory>("/" + robot_ns + publisherTopicName + "/command", 1);
  //   jntCmdPublisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/" + robot_ns + publisherTopicName + "/command", rclcpp::QoS(1));
  // }
  // else if (publisher == PublisherType::TrajectoryAction)
  //   jntCmdPublisher = nh.advertise<control_msgs::msg::FollowJointTrajectoryActionGoal>("/" + robot_ns + publisherTopicName + "/follow_joint_trajectory/goal", 1);
  // else
  // jntCmdPublisher = nh.advertise<std_msgs::msg::Float64MultiArray>("/" + robot_ns + publisherTopicName + "/command", 2);
  jntCmdPublisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/" + robot_ns + publisherTopicName + "/commands", rclcpp::QoS(1));

  desStatePublisher = node->create_publisher<ohrc_msgs::msg::State>("/" + robot_ns + "state/desired", rclcpp::QoS(100));
  curStatePublisher = node->create_publisher<ohrc_msgs::msg::State>("/" + robot_ns + "state/current", rclcpp::QoS(100));

  if (robot_ns != "")
    service = node->create_service<std_srvs::srv::Empty>("/" + robot_ns + "reset", std::bind(&CartController::resetService, this, _1, _2));

  Affine3d T_init_base = getTransform_base(robot_ns + initPoseFrame);
  T_init = Translation3d(initPose[0], initPose[1], initPose[2]) *
           (AngleAxisd(initPose[3], Vector3d::UnitX()) * AngleAxisd(initPose[4], Vector3d::UnitY()) * AngleAxisd(initPose[5], Vector3d::UnitZ()));
  T_init = T_init_base * T_init;

  for (int i = 0; i < 6; i++) {
    posFilter.push_back(butterworth(2, freq / 10.0, freq));
    velFilter.push_back(butterworth(2, freq / 20.0, freq));
  }
  for (int i = 0; i < nJnt; i++)
    jntFilter.push_back(butterworth(2, freq / 50.0, freq));
}

bool CartController::getRosParams() {
  RclcppUtility::declare_and_get_parameter(this->node, hw_config_ns + "chain_start", std::string(""), chain_start);
  RclcppUtility::declare_and_get_parameter(this->node, hw_config_ns + "chain_end", std::string(""), chain_end);

  if (chain_start == "" || chain_end == "") {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Missing chain info in launch file. chain_start: " << chain_start << ", chain_end: " << chain_end);
    return false;
  }

  if (!RclcppUtility::declare_and_get_parameter_enum(this->node, "solver", SolverType::MyIK, solver))
    return false;

  if (!RclcppUtility::declare_and_get_parameter_enum(this->node, hw_config_ns + "publisher", PublisherType::Velocity, publisher))
    return false;

  std::string defaltPublisherTopicName;
  if (publisher == PublisherType::Position)
    defaltPublisherTopicName = "joint_position_controller";
  else if (publisher == PublisherType::Velocity)
    defaltPublisherTopicName = "joint_velocity_controller";
  else if (publisher == PublisherType::Trajectory || publisher == PublisherType::TrajectoryAction)
    defaltPublisherTopicName = "joint_trajectory_controller";
  RclcppUtility::declare_and_get_parameter(this->node, hw_config_ns + "topic_namespace", defaltPublisherTopicName, publisherTopicName);

  double freq_bound;
  if (RclcppUtility::declare_and_get_parameter(this->node, hw_config_ns + "freq_bound", 0.0, freq_bound, false) && freq_bound < freq && freq_bound > 0.0) {
    RCLCPP_WARN_STREAM(node->get_logger(), "freq is bounded by " << freq_bound << " for " << hw_config_ns);
    freq = freq_bound;
  }

  RclcppUtility::declare_and_get_parameter(this->node, hw_config_ns + "initial_pose_frame", chain_start, initPoseFrame);

  RclcppUtility::declare_and_get_parameter(this->node, hw_config_ns + "initial_pose", std::vector<double>{ 0.45, 0.0, 0.85, 0.0, M_PI, -M_PI_2 }, initPose);

  RclcppUtility::declare_and_get_parameter(this->node, hw_config_ns + "initIKAngle", std::vector<double>(nJnt, 0.0), _q_init_expect);

  return true;
}

void CartController::initMembers() {
  if (root_frame == "")
    root_frame = chain_start;

  dt = 1.0 / freq;

  // this->T_base_root = trans->getTransform(root_frame, robot_ns + chain_start, rclcpp::Time(0), rclcpp::Duration(1, 0));
  myik_solver_ptr = std::make_shared<MyIK::MyIK>(this->node, chain_start, chain_end, urdf_param, eps, T_base_root);
  bool valid = myik_solver_ptr->getKDLChain(chain);
  chain_segs = chain.segments;

  nJnt = chain.getNrOfJoints();
  _q_cur.resize(nJnt);
}

void CartController::updatePosFilterCutoff(const double posFreq) {
  for (int i = 0; i < 6; i++)
    posFilter[i] = butterworth(2, posFreq, freq);
}

void CartController::updateVelFilterCutoff(const double velFreq) {
  for (int i = 0; i < 6; i++)
    velFilter[i] = butterworth(2, velFreq, freq);
}

void CartController::updateJntFilterCutoff(const double jntFreq) {
  for (int i = 0; i < nJnt; i++)
    jntFilter[i] = butterworth(2, jntFreq, freq);
}

void CartController::updateFilterCutoff(const double velFreq, const double jntFreq) {
  this->updateVelFilterCutoff(velFreq);
  this->updateJntFilterCutoff(jntFreq);
}

CartController::~CartController() {
}

bool CartController::resetService(const std::shared_ptr<std_srvs::srv::Empty::Request> req, const std::shared_ptr<std_srvs::srv::Empty::Response>& res) {
  this->resetPose();
  this->resetFt();
  return true;
}

void CartController::resetPose() {
  // ROS_WARN_STREAM("The robot will moves to the initail pose!");
  RCLCPP_WARN_STREAM(node->get_logger(), "The robot will moves to the initial pose!");

  s_cbJntState.isFirst = true;
  initialized = false;
  s_moveInitPos.isFirst = true;

  while (!isInitialized() && rclcpp::ok()) {
    rclcpp::sleep_for(100ms);
  }
}

Affine3d CartController::getTransform_base(std::string target) {
  return trans->getTransform(robot_ns + chain_start, target, rclcpp::Time(0), rclcpp::Duration(1.0, 0));
}

// void CartController::signal_handler(int signum) {
//   // ros::NodeHandle nh;
//   // std::vector<std::string> robot_ns{ "/toroboarm_1", "/toroboarm_2" };
//   // std::vector<std::string> controller{ "/joint_position_controller", "/joint_velocity_controller" };
//   // ros::Publisher publisher;
//   // std_msgs::Float64MultiArray cmd;
//   // cmd.data.resize(7, 0.0);
//   // while (ros::ok()) {  // exerimental
//   //   for (int i = 0; i < robot_ns.size(); i++) {
//   //     for (int j = 0; j < controller.size(); i++) {
//   //       publisher = nh.advertise<std_msgs::Float64MultiArray>(robot_ns[i] + controller[j] + "/command", 1);
//   //       publisher.publish(cmd);
//   //     }
//   //   }
//   //   ros::shutdown();
//   // }
// }
void CartController::resetFt() {
  // ROS_INFO_STREAM("Called reset ft service.");
  RCLCPP_INFO_STREAM(node->get_logger(), "Called reset ft service.");

  // std_srvs::srv::Empty srv;
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  client->async_send_request(request);
}

void CartController::initWithJnt(const KDL::JntArray& q_init) {
  // this->resetFt();
}

void CartController::getVelocity(const KDL::Frame& frame, const KDL::Frame& prev_frame, const double& dt, KDL::Twist& twist) const {
  Eigen::Affine3d T, T_prev;
  VectorXd vel(6);
  tf2::transformKDLToEigen(frame, T);
  tf2::transformKDLToEigen(prev_frame, T_prev);

  vel.head(3) = (T.translation() - T_prev.translation()) / dt;
  vel.tail(3) = rotation_util::getQuaternionError(Quaterniond(T.rotation()), Quaterniond(T_prev.rotation())) / dt;

  // std::cout << vel.transpose() << std::endl;

  tf2::twistEigenToKDL(vel, twist);
  // vel.tail(3) = rotation_util::getQuaternionError(Quaterniond(T.rotation()), Quaterniond(T_prev.rotation())) / dt;
}

void CartController::cbJntState(const sensor_msgs::msg::JointState::SharedPtr msg) {
  // std::cout << "cbJntState" << std::endl;
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

// void CartController::cbArmMarker(const visualization_msgs::msg::MarkerArray::SharedPtr& msg) {
//   int nMarker = msg->markers.size();

//   for (int i = 0; i < nMarker; i++) {
//     if (msg->markers[i].id == ArmMarker::ArmMarkerID::ManipulabilityEllipsoid) {
//       Quaterniond q;
//       tf2::fromMsg(msg->markers[i].pose.orientation, q);

//       std::lock_guard<std::mutex> lock(mtx);
//       _userManipU = q.matrix();
//       flagArmMarker = true;
//     }
//   }
// }

void CartController::cbForce(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx);
  _force.header.stamp = msg->header.stamp;
  _force.header.frame_id = robot_ns + chain_end;
  _force.wrench = geometry_msgs_utility::transformFT(msg->wrench, Tft_eff);

  this->pubEefForce->publish(_force);

  if (!flagForce)
    flagForce = true;
}

void CartController::initDesWithJnt(const KDL::JntArray& q_cur) {
  std::lock_guard<std::mutex> lock(mtx);
  fk_solver_ptr->JntToCart(q_cur, this->_des_eef_pose);
  this->_des_eef_vel = KDL::Twist::Zero();
}

int CartController::moveInitPos(const KDL::JntArray& q_cur, const std::vector<std::string> nameJnt, std::vector<int> idxSegJnt) {
  // ROS_INFO_STREAM_ONCE("Moving initial posiiton");
  RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "Moving initial position");

  if (s_moveInitPos.isFirst) {
    this->nameJnt = nameJnt;
    s_moveInitPos.q_initial = q_cur;
    KDL::Frame init_eef_pose;

    tf2::transformEigenToKDL(T_init, init_eef_pose);

    KDL::JntArray q_init_expect;
    q_init_expect.resize(nJnt);
    q_init_expect.data = VectorXd::Map(&_q_init_expect[0], nJnt);

    int rc;
    switch (solver) {
        // case SolverType::Trac_IK:
        //   rc = tracik_solver_ptr->CartToJnt(q_init_expect, init_eef_pose, s_moveInitPos.q_des);
        //   break;

        // case SolverType::KDL:
        //   rc = kdl_solver_ptr->CartToJnt(q_init_expect, init_eef_pose, s_moveInitPos.q_des);
        //   break;

      case SolverType::MyIK:
        myik_solver_ptr->setNameJnt(nameJnt);
        myik_solver_ptr->setIdxSegJnt(idxSegJnt);
        rc = myik_solver_ptr->CartToJnt(q_init_expect, init_eef_pose, s_moveInitPos.q_des, 5.0);
        break;
    }

    if (rc < 0) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to find initial joint angle. Please check if the initial position is appropriate.");
      return false;
    } else
      RCLCPP_INFO_STREAM(node->get_logger(), "Successfuly solved initial joint angles.");

    s_moveInitPos.isFirst = false;

    s_moveInitPos.t_s = node->get_clock()->now();

    this->q_rest = s_moveInitPos.q_des;
  }

  const double T = 10.0;
  // static rclcpp::Time s_moveInitPos.t_s = rclcpp::Time::now();

  bool lastLoop = false;
  double s = 0.0, s2, s3, s4, s5;
  s = (node->get_clock()->now() - s_moveInitPos.t_s).nanoseconds() * 1.0e-9 / T;
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
      // ROS_ERROR_STREAM_ONCE("Torque controller is still not implemented...");
      RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), "Torque controller is still not implemented...");
      break;
    case PublisherType::Trajectory:
      sendTrajectoryCmd(s_moveInitPos.q_des.data, T * (1.0 - s));
      break;
    case PublisherType::TrajectoryAction:
      sendTrajectoryActionCmd(s_moveInitPos.q_des.data, T * (1.0 - s));
      // rclcpp::Duration(T).sleep();
      // rclcpp::sleep_for(T.second())
      lastLoop = true;
      break;
    default:
      // ROS_ERROR_STREAM_ONCE("This publisher is not implemented...");
      RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), "This publisher is not implemented...");
      break;
  }

  if (!lastLoop)
    return false;

  return true;
}

void CartController::sendPositionCmd(const VectorXd& q_des) {
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = std::vector<double>(q_des.data(), q_des.data() + q_des.rows() * q_des.cols());
  jntCmdPublisher->publish(cmd);
}

void CartController::sendVelocityCmd(const VectorXd& dq_des) {
  sendPositionCmd(dq_des);
}

void CartController::sendVelocityCmd(const VectorXd& q_des, const VectorXd& dq_des, const KDL::JntArray& q_cur, const bool& lastLoop) {
  double kp = 4.0;  // feedback p gain
  std_msgs::msg::Float64MultiArray cmd;
  VectorXd dq_des_ = dq_des + (1.0 - (double)lastLoop) * kp * (q_des - q_cur.data);
  sendVelocityCmd(dq_des_);
}

void CartController::getTrajectoryCmd(const VectorXd& q_des, const double& T, trajectory_msgs::msg::JointTrajectory& cmd_trj) {
  cmd_trj.header.stamp = node->get_clock()->now();
  cmd_trj.points.resize(1);
  cmd_trj.points[0].time_from_start = rclcpp::Duration(T, 0);  //??

  for (int i = 0; i < nJnt; i++) {
    cmd_trj.joint_names.push_back(nameJnt[i]);
    cmd_trj.points[0].positions.push_back(q_des[i]);
    cmd_trj.points[0].velocities.push_back(0.0);
    cmd_trj.points[0].accelerations.push_back(0.0);
  }
}

void CartController::getTrajectoryCmd(const VectorXd& q_des, const VectorXd& dq_des, const double& T, trajectory_msgs::msg::JointTrajectory& cmd_trj) {
  // cmd_trj.header.stamp = rclcpp::Time::now();
  cmd_trj.points.resize(1);
  cmd_trj.points[0].time_from_start = rclcpp::Duration(T, 0);

  for (int i = 0; i < nJnt; i++) {
    cmd_trj.joint_names.push_back(nameJnt[i]);
    cmd_trj.points[0].positions.push_back(q_des[i]);
    cmd_trj.points[0].velocities.push_back(dq_des[i]);
    cmd_trj.points[0].accelerations.push_back(0.0);
  }
}
void CartController::sendTrajectoryCmd(const VectorXd& q_des, const double& T) {
  // trajectory_msgs::msg::JointTrajectory cmd_trj;
  // getTrajectoryCmd(q_des, T, cmd_trj);
  // jntCmdPublisher->publish(cmd_trj);
}

void CartController::sendTrajectoryCmd(const VectorXd& q_des, const VectorXd& dq_des, const double& T) {
  // trajectory_msgs::msg::JointTrajectory cmd_trj;
  // getTrajectoryCmd(q_des, dq_des, T, cmd_trj);
  // jntCmdPublisher->publish(cmd_trj);
}

void CartController::sendTrajectoryActionCmd(const VectorXd& q_des, const double& T) {
  // control_msgs::msg::FollowJointTrajectoryActionGoal cmd_trjAction;
  // getTrajectoryCmd(q_des, T, cmd_trjAction.goal.trajectory);
  // jntCmdPublisher->publish(cmd_trjAction);
}
void CartController::sendTrajectoryActionCmd(const VectorXd& q_des, const VectorXd& dq_des, const double& T) {
  // control_msgs::FollowJointTrajectoryActionGoal cmd_trjAction;
  // getTrajectoryCmd(q_des, dq_des, T, cmd_trjAction.goal.trajectory);
  // cmd_trjAction.header.stamp = cmd_trjAction.goal.trajectory.header.stamp;
  // jntCmdPublisher.publish(cmd_trjAction);
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

  static rclcpp::Time t0 = rclcpp::Time::now();
  if (disable)
    t0 = rclcpp::Time::now();

  double s = (rclcpp::Time::now() - t0).toSec() / 2.0;
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
  tf2::twistKDLToEigen(des_eef_vel, vel);
  for (int i = 0; i < 6; i++)
    vel(i) = velFilter[i].filter(vel(i));

  tf2::twistEigenToKDL(vel, des_eef_vel);
}

void CartController::publishState(const KDL::Frame& pose, const KDL::Twist& vel, rclcpp::Publisher<ohrc_msgs::msg::State>::SharedPtr publisher) {
  this->publishState(pose, vel, geometry_msgs::msg::Wrench(), publisher);
}

void CartController::publishState(const KDL::Frame& pose, const KDL::Twist& vel, const geometry_msgs::msg::Wrench& wrench,
                                  rclcpp::Publisher<ohrc_msgs::msg::State>::SharedPtr publisher) {
  ohrc_msgs::msg::State state;
  state.header.stamp = node->get_clock()->now();
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
  publishState(des_eef_pose, des_eef_vel, desStatePublisher);
}

void CartController::publishCurEffPoseVel(const KDL::Frame& cur_eef_pose, const KDL::Twist& cur_eef_vel) {
  std::lock_guard<std::mutex> lock(mtx);
  publishState(cur_eef_pose, cur_eef_vel, _force.wrench, curStatePublisher);
}

void CartController::publishMarker(const KDL::JntArray q_cur) {
  // visualization_msgs::Marker manipuMarker = myik_solver_ptr->getManipulabilityMarker(q_cur);
  // manipuMarker.header.frame_id = robot_ns + chain_start;

  // visualization_msgs::MarkerArray markers;
  // markers.markers.push_back(manipuMarker);

  // markerPublisher.publish(markers);
}

/**
 * \brief Starts controller
 * \param time Current time
 */
void CartController::starting(const rclcpp::Time& time) {
  // check Gazebo is ready
  // if (!gazebo_utility::checkGazeboInit())
  // return;
  // std::cout << __FILE__ << " " << __LINE__ << std::endl;
  // start to subscribe topics
  // spinner_->start();
  // spinner->start();
  // std::cout << __FILE__ << " " << __LINE__ << std::endl;
  // wait for subscribing registered topics

  subscriber_utility::checkSubTopic(node, subFlagPtrs, &mtx, robot_ns);

  this->resetFt();

  updateCurState();
}

/**
 * \brief Stops controller
 * \param time Current time
 */
void CartController::stopping(const rclcpp::Time& /*time*/) {
  if (controller == ControllerType::Velocity) {
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.resize(nJnt, 0.0);
    jntCmdPublisher->publish(cmd);
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
  tf2::transformKDLToEigen(cur_pose, T);
  tf2::transformKDLToEigen(des_pose, Td);

  static rclcpp::Time t0 = node->get_clock()->now();
  if (disable) {
    t0 = node->get_clock()->now();
    disablePoseFeedback();  // no longer used
  } else
    enablePoseFeedback();  // no longer used

  double s = (node->get_clock()->now() - t0).nanoseconds() * 1.0e-9 / 3.0;
  if (s > 1.0 || passThrough)
    s = 1.0;  // 0-1

  Td.translation() = s * (Td.translation() - T.translation()) + T.translation();
  Td.linear() = Quaterniond(T.rotation()).slerp(s, Quaterniond(Td.rotation())).toRotationMatrix();

  tf2::transformEigenToKDL(Td, des_pose);

  // KDL::Twist twist;
  Matrix<double, 6, 1> v, vd;
  v << 0, 0, 0, 0, 0, 0;
  // tf::twistKDLToEigen(cur_vel, v);
  tf2::twistKDLToEigen(des_vel, vd);

  vd = s * (vd - v) + v;
  tf2::twistEigenToKDL(vd, des_vel);
}

/**
 * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
 * \param time   Current time
 * \param period Time since the last called to update
 */
void CartController::update() {
  update(node->get_clock()->now(), rclcpp::Duration(1.0 / freq, 0));
}
void CartController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  if ((time - prev_time) < period)
    return;

  prev_time = time;

  updateCurState();
  // std::cout << robot_ns << std::endl;
  double dt = period.nanoseconds() * 1e-9;

  // ROS_INFO_STREAM_ONCE("Start teleoperation");
  RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "Start teleoperation");
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
      // case SolverType::Trac_IK:
      //   rc = tracik_solver_ptr->CartToJnt(q_cur, des_eef_pose, q_des);
      //   break;
      // case SolverType::KDL:
      //   rc = kdl_solver_ptr->CartToJnt(q_cur, des_eef_pose, q_des);
      //   break;
      case SolverType::MyIK:
        rc = myik_solver_ptr->CartToJnt(q_cur, des_eef_pose, q_des, dt);
        break;
    }
    if (rc < 0) {
      // ROS_WARN_STREAM("Failed to solve IK. Skip this control loop");
      RCLCPP_WARN_STREAM(node->get_logger(), "Failed to solve IK. Skip this control loop");
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
      // ROS_WARN_STREAM("Failed to solve IK. Skip this control loop");
      RCLCPP_WARN_STREAM(node->get_logger(), "Failed to solve IK. Skip this control loop");
      return;
    }

    // low pass filter
    filterJnt(dq_des);

    q_des.data += dq_des.data * dt;

    sendVelCmd(q_des, dq_des, dt);

  } else if (controller == ControllerType::Torque) {
    // torque controller
  } else
    // ROS_ERROR_STREAM("Not Implemented!");
    RCLCPP_ERROR_STREAM(node->get_logger(), "Not Implemented!");
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
      // ROS_ERROR_STREAM_ONCE("This controller is not implemented...");
      RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), "This controller is not implemented...");
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
      // ROS_ERROR_STREAM_ONCE("This controller is not implemented...");
      RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), "This controller is not implemented...");
      break;
  }
}

void CartController::filterJnt(KDL::JntArray& q) {
  for (int i = 0; i < nJnt; i++)
    q.data(i) = jntFilter[i].filter(q.data(i));
}

int CartController::control() {
  starting(node->get_clock()->now());
  rclcpp::sleep_for(3s);

  while (rclcpp::ok())
    update(node->get_clock()->now(), rclcpp::Duration(1.0 / freq, 0));

  stopping(node->get_clock()->now());

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