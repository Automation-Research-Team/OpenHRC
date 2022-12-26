#include "ohrc_control/forwarding_controller.hpp"

ForwardingController::ForwardingController() : nh("~"), spinner(0) {
  std::signal(SIGINT, ForwardingController::signal_handler);

  nh.param("num_samples", num_samples, 1000);
  nh.param("chain_start", chain_start, std::string(""));
  nh.param("chain_end", chain_end, std::string(""));

  if (chain_start == "" || chain_end == "") {
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }

  // nh.param("timeout", timeout, 0.005);
  // timeout = 1.0 / freq;

  robot_ns.resize(2);
  nh.param("robot_ns", robot_ns[RobotType::Follower], std::string(""));
  // nh.param("urdf_param", urdf_param, std::string("/toroboarm/robot_description"));
  robot_ns[RobotType::Follower] = robot_ns[RobotType::Follower] + "/";

  nh.param("interface_robot_ns", robot_ns[RobotType::Leader], std::string(""));
  // nh.param("urdf_param", urdf_param, std::string("/toroboarm/robot_description"));
  robot_ns[RobotType::Leader] = robot_ns[RobotType::Leader] + "/";

  urdf_param = "/" + robot_ns[RobotType::Follower] + "robot_description";

  myik_solver_ptr.reset(new MyIK::MyIK(chain_start, chain_end, urdf_param, eps));
  bool valid = myik_solver_ptr->getKDLChain(chain);
  chain_segs = chain.segments;

  nJnt = chain.getNrOfJoints();
  _q_cur.resize(nJnt);

  std::vector<void (ForwardingController::*)(const sensor_msgs::JointState::ConstPtr&)> cbFunc;
  cbFunc.push_back(&ForwardingController::cbJntState);
  cbFunc.push_back(&ForwardingController::cbIfJntState);

  jntStateSubscriber.resize(nJnt);
  jntPosCmdPublisher.resize(nJnt);
  jntVelCmdPublisher.resize(nJnt);

  for (int i = 0; i < 2; i++) {
    jntStateSubscriber[i] = nh.subscribe("/" + robot_ns[i] + "joint_states", 2, cbFunc[i], this, ros::TransportHints().tcpNoDelay());

    jntPosCmdPublisher[i] = nh.advertise<std_msgs::Float64MultiArray>("/" + robot_ns[i] + "joint_position_controller/command", 2);
    jntVelCmdPublisher[i] = nh.advertise<std_msgs::Float64MultiArray>("/" + robot_ns[i] + "joint_velocity_controller/command", 2);
  }

  std::string solver_str;
  if (!nh.param("solver", solver_str, std::string("MyIK")))
    ROS_WARN_STREAM("Solver type is not choisen {Trac_Ik, KDL, MyIK}: Default MyIK");
  else
    ROS_INFO_STREAM("Solver: " << solver_str);

  if (solver_str == "Trac_IK")
    solver = SolverType::Trac_IK;
  else if (solver_str == "KDL")
    solver = SolverType::KDL;
  else if (solver_str == "MyIK")
    solver = SolverType::MyIK;
  else {
    ROS_FATAL("Solver type is not correctly choisen from {Trac_IK, KDL, MyIK}");
    exit(-1);
  }

  std::string controller_str;
  if (!nh.param("controller", controller_str, std::string("Position")))
    ROS_WARN_STREAM("Controller is not choisen {Position, Velocity, Torque}: Default Position");
  else
    ROS_INFO_STREAM("Controller: " << controller_str);

  if (controller_str == "Position")
    controller = ControllerType::Position;
  else if (controller_str == "Velocity")
    controller = ControllerType::Velocity;
  else if (controller_str == "Torque")
    controller = ControllerType::Torque;
  else {
    ROS_FATAL("Controller type is not correctly choisen from {Position, Velocity, Torque}");
    exit(-1);
  }

  subFlagPtrs.push_back(&flagJntState);
  subFlagPtrs.push_back(&flagIfJntState);

  T_init = Translation3d(0.45, 0.0, 0.85) * AngleAxisd(M_PI, Vector3d::UnitY());
}

ForwardingController::~ForwardingController() {
}

void ForwardingController::signal_handler(int signum) {
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

bool ForwardingController::getJntState(const sensor_msgs::JointState::ConstPtr& msg, KDL::JntArray& q_cur, KDL::JntArray& dq_cur, RobotType type, bool& isFirst, bool& initialized, bool& isFirst2, KDL::JntArray& q_des,
                                       KDL::JntArray& q_init) {
  unsigned int j = 0;
  for (size_t i = 0; i < chain_segs.size(); i++) {
    auto result = std::find(msg->name.begin(), msg->name.end(), chain_segs[i].getJoint().getName());

    if (result == msg->name.end())
      continue;

    q_cur.data[j] = msg->position[std::distance(msg->name.begin(), result)];
    dq_cur.data[j] = msg->velocity[std::distance(msg->name.begin(), result)];
    j++;
  }

  // std::cout << j << std::endl;
  if (j != nJnt)
    return false;

  // if (isFirst) {
  //   // if (!initialized) {
  //   //   initialized = moveInitPos(q_cur, type, isFirst2, q_des, q_init);
  //   //   // initialiuzed = moveInitPos();
  //   //   return false;
  //   // }
  //   initialized = true;

  //   // initDesWithJnt(q_cur);
  //   // initWithJnt(q_cur);
  //   isFirst = false;
  //   return false;
  // }

  return true;
}

void ForwardingController::cbJntState(const sensor_msgs::JointState::ConstPtr& msg) {
  KDL::JntArray q_cur(chain.getNrOfJoints()), dq_cur(chain.getNrOfJoints());

  static bool isFirst = true;
  static bool initialized = false;

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

  if (isFirst) {
    if (!initialized) {
      initialized = moveInitJntPos(q_cur);
      // initialiuzed = moveInitPos();
      return;
    }

    // initDesWithJnt(q_cur);
    // initWithJnt(q_cur);
    isFirst = false;
    return;
  }

  std::lock_guard<std::mutex> lock(mtx);
  _q_cur = q_cur;
  _dq_cur = dq_cur;
  if (!flagJntState)
    flagJntState = true;
}

void ForwardingController::cbIfJntState(const sensor_msgs::JointState::ConstPtr& msg) {
  KDL::JntArray q_cur(chain.getNrOfJoints()), dq_cur(chain.getNrOfJoints());

  unsigned int j = 0;
  for (size_t i = 0; i < chain_segs.size(); i++) {
    auto result = std::find(msg->name.begin(), msg->name.end(), chain_segs[i].getJoint().getName());

    if (result == msg->name.end())
      continue;

    q_cur.data[j] = msg->position[std::distance(msg->name.begin(), result)];
    dq_cur.data[j] = msg->velocity[std::distance(msg->name.begin(), result)];
    j++;
  }

  // std::cout << j << std::endl;
  if (j != nJnt)
    return;

  // std::cout << q_cur.data.transpose() << std::endl;
  std::lock_guard<std::mutex> lock(mtx);
  _q_cur_if = q_cur;
  _dq_cur_if = dq_cur;

  _q_des = q_cur;
  if (!flagIfJntState)
    flagIfJntState = true;
}

void ForwardingController::initDesWithJnt(const KDL::JntArray& q_cur) {
  std::lock_guard<std::mutex> lock(mtx);
  fk_solver_ptr->JntToCart(q_cur, this->_des_eff_pose);
  // this->_des_eff_vel;
}

int ForwardingController::moveInitJntPos(const KDL::JntArray& q_cur) {
  ROS_INFO_STREAM_ONCE("Moving initial posiiton");

  if (_q_des.data.size() != nJnt)
    return false;

  static KDL::JntArray q_des = _q_des;
  static KDL::JntArray q_init = q_cur;

  const double T = 10.0;
  static ros::Time t_s = ros::Time::now();

  bool lastLoop = false;
  double s = 0.0, s2, s3, s4, s5;
  s = (ros::Time::now() - t_s).toSec() / T;
  if (s > 1.0) {
    s = 1.0;
    lastLoop = true;
  }
  s2 = s * s;
  s3 = s2 * s;
  s4 = s3 * s;
  s5 = s4 * s;

  // min jerk trajectory
  VectorXd q_des_t = q_init.data + (q_des.data - q_init.data) * (6.0 * s5 - 15.0 * s4 + 10.0 * s3);
  VectorXd dq_des_t = (q_des.data - q_init.data) * (30.0 * s4 - 60.0 * s3 + 30.0 * s2) / T;

  std_msgs::Float64MultiArray cmd;
  if (controller == ControllerType::Position) {
    cmd.data = std::vector<double>(q_des_t.data(), q_des_t.data() + q_des_t.rows() * q_des_t.cols());
    jntPosCmdPublisher[RobotType::Follower].publish(cmd);
  } else if (controller == ControllerType::Velocity) {
    VectorXd dq_des_t_fb = dq_des_t + (double)(!lastLoop) * 1.0 * (q_des_t - q_cur.data);
    cmd.data = std::vector<double>(dq_des_t_fb.data(), dq_des_t_fb.data() + dq_des_t_fb.rows() * dq_des_t_fb.cols());
    jntVelCmdPublisher[RobotType::Follower].publish(cmd);
  } else if (controller == ControllerType::Torque) {
    // torque controller
  }

  if (!lastLoop)
    return false;

  return true;
}
void ForwardingController::getDesEffPoseVel(const double& dt, const KDL::JntArray& q_cur, KDL::Frame& des_eff_pose, KDL::Twist& des_eff_vel) {
  std::lock_guard<std::mutex> lock(mtx);
  des_eff_pose = this->_des_eff_pose;
  des_eff_vel = this->_des_eff_vel;
}

// void ForwardingController::publishDesEffPoseVel(const KDL::Frame& des_eff_pose, const KDL::Twist& des_eff_vel) {
//   static geometry_msgs::TransformStamped transform;
//   if ((ros::Time::now() - transform.header.stamp).toSec() < 1.0 / 50.0)
//     return;

//   transform = tf2::kdlToTransform(des_eff_pose);
//   transform.header.stamp = ros::Time::now();

//   transform.header.frame_id = robot_ns + chain_start;
//   transform.child_frame_id = robot_ns + chain_end + "_d";
//   br.sendTransform(transform);
// }

int ForwardingController::control() {
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
  KDL::JntArray q_cur, q_cur_if, dq_cur, dq_cur_if;
  std_msgs::Float64MultiArray cmd;
  int rc;

  ROS_INFO_STREAM("Start teleoperation");
  while (ros::ok()) {
    {
      std::lock_guard<std::mutex> lock(mtx);
      q_cur = this->_q_cur;
      dq_cur = this->_dq_cur;
      q_cur_if = this->_q_cur_if;
      dq_cur_if = this->_dq_cur_if;
    }

    if (controller == ControllerType::Position) {
      KDL::JntArray q_des(nJnt);
      q_des = q_cur_if;

      cmd.data = std::vector<double>(q_des.data.data(), q_des.data.data() + q_des.data.rows() * q_des.data.cols());
      jntPosCmdPublisher[RobotType::Follower].publish(cmd);
      // ROS_INFO_STREAM(cmd);

    } else if (controller == ControllerType::Velocity) {
      KDL::JntArray dq_des(nJnt), q_des(nJnt);
      dq_des = dq_cur_if;
      q_des = q_cur_if;

      VectorXd v = VectorXd::Zero(7);
      v = dq_des.data + (q_des.data - q_cur.data);

      cmd.data = std::vector<double>(v.data(), v.data() + v.rows() * v.cols());
      jntVelCmdPublisher[RobotType::Follower].publish(cmd);
    } else if (controller == ControllerType::Torque) {
      // torque controller
    }

    r.sleep();
  }

  return 1;
}

// int main(int argc, char** argv) {
//   ros::init(argc, argv, "marker_teleoperation");
//   ForwardingController ForwardingController;
//   if (ForwardingController.control() < 0)
//     ROS_ERROR("End by some fails");
// }
