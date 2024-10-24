#include "ohrc_control/multi_cart_controller.hpp"

Controller::Controller() : Node("ohrc_controller") {
  // std::cout << "Controller started??" << std::endl;
  // this->node = std::shared_ptr<rclcpp::Node>(this);

  // std::vector<std::string> robots, hw_configs;
  // if (!this->getRosParams(robots, hw_configs)) {
  //   RCLCPP_FATAL_STREAM(this->get_logger(), "Failed to get the initial parameters. Shutting down...");
  //   rclcpp::shutdown();
  // }

  // this->initMenbers(robots, hw_configs);

  // control_timer = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&Controller::controlLoop, this));
}

Controller::~Controller() {
  this->stopping();
}

bool Controller::getRosParams(std::vector<std::string>& robots, std::vector<std::string>& hw_configs) {
  std::vector<std::string> _hw_configs;
  if (!RclcppUtility::declare_and_get_parameter(this->node, "follower.hw", std::vector<std::string>(), _hw_configs, false) || _hw_configs.empty() ||
      std::all_of(_hw_configs.begin(), _hw_configs.end(), [](std::string x) { return x == ""; })) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Failed to get the follower robot hw list");
    return false;
  }

  for (std::string hw_config : _hw_configs) {
    std::vector<std::string> _robots;

    if (!RclcppUtility::declare_and_get_parameter(this->node, "follower.ns." + hw_config, std::vector<std::string>(), _robots, false) || _robots.empty()) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Failed to get the follower robot ns list");
      return false;
    }

    for (auto robot : _robots) {
      robots.push_back(robot);
      hw_configs.push_back(hw_config);
    }
  }

  for (int i = 0; i < robots.size(); i++)
    RCLCPP_INFO_STREAM(this->get_logger(), "Configured Robot " << i << ": " << hw_configs[i] << " (ns: " << robots[i] << ")");

  if (!RclcppUtility::declare_and_get_parameter(this->node, "root_frame", std::string(""), root_frame))
    return false;

  if (!RclcppUtility::declare_and_get_parameter(this->node, "control_freq", double(0.0), freq))
    return false;

  if (!RclcppUtility::declare_and_get_parameter_enum(this->node, "controller", ControllerType::Velocity, controller))
    return false;

  // if (!RclcppUtility::declare_and_get_parameter_enum(node, "publisher", PublisherType::Velocity, publisher))
  //   return false;

  if (!RclcppUtility::declare_and_get_parameter_enum(this->node, "feedback_mode", FeedbackMode::PositionFeedback, feedbackMode))
    return false;

  if (!RclcppUtility::declare_and_get_parameter_enum(this->node, "MF_mode", MFMode::Individual, MFmode))
    return false;

  if (!RclcppUtility::declare_and_get_parameter_enum(this->node, "IK_mode", IKMode::Concatenated, IKmode))
    return false;

  if (!RclcppUtility::declare_and_get_parameter_enum(this->node, "priority", PriorityType::Manual, priority))
    return false;

  // if (priority == PriorityType::Adaptation)
  //   n.param<std::string>("adaptation_option", adaptationOption_, "Default");

  // if (!n.getParam("date", this->date)) {
  //   this->date = std_utility::getDatetimeStr();
  //   n.setParam("date", date);
  // }

  return true;
}

void Controller::initMenbers(const std::vector<std::string> robots, const std::vector<std::string> hw_configs) {
  nRobot = robots.size();
  this->dt = 1.0 / this->freq;

  prev_time.resize(nRobot, this->get_clock()->now());
  cartControllers.resize(nRobot);
  nodes.resize(nRobot);

  for (int i = 0; i < nRobot; i++) {
    cartControllers[i] = std::make_shared<CartController>(nodes[i], robots[i], hw_configs[i], root_frame, i, controller, freq);  // TODO: might can be replaced with sub_node
  }

  std::vector<std::string> base_link(nRobot), tip_link(nRobot);  // TODO: base_links for all robot shoud be same
  std::vector<Affine3d> T_base_root(nRobot);
  myik_ptr.resize(nRobot);
  for (int i = 0; i < nRobot; i++)
    cartControllers[i]->getInfo(base_link[i], tip_link[i], T_base_root[i], myik_ptr[i]);

  multimyik_solver_ptr = std::make_unique<MyIK::MyIK>(node, base_link, tip_link, T_base_root, myik_ptr);

  // service = nh.advertiseService("/reset", &Controller::resetService, this);
  service = this->create_service<std_srvs::srv::Empty>("/reset", std::bind(&Controller::resetService, this, _1, _2));

  // TODO: condifure this priority setting

  for (int i = 0; i < nRobot; i = i + 2)
    manualInd.push_back(i);

  for (int i = 1; i < nRobot; i = i + 2)
    autoInd.push_back(i);

  setPriority(priority);

  desPose.resize(nRobot);
  desVel.resize(nRobot);

  interfaces.resize(nRobot);

  baseControllers.resize(nRobot);
  for (int i = 0; i < nRobot; i++) {
    if (cartControllers[i]->getFtFound() && feedbackMode == FeedbackMode::Admittance)
      baseControllers[i] = std::make_shared<AdmittanceController>(cartControllers[i]);
    else if (feedbackMode == FeedbackMode::PositionFeedback)
      baseControllers[i] = std::make_shared<PositionFeedbackController>(cartControllers[i]);
    else if (feedbackMode == FeedbackMode::HybridFeedback)
      baseControllers[i] = std::make_shared<HybridFeedbackController>(cartControllers[i]);

    cartControllers[i]->disablePoseFeedback();  // TODO: Pose feedback would be always enable. original feedback code can be removed.
  }

  this->defineInterface();

  control_timer = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&Controller::controlLoop, this));
}

void Controller::resetService(const std::shared_ptr<std_srvs::srv::Empty::Request> req, const std::shared_ptr<std_srvs::srv::Empty::Response>& res) {
  // ROS_INFO_STREAM("Resetting...");
  RCLCPP_INFO_STREAM(this->get_logger(), "Resetting...");
  for (int i = 0; i < nRobot; i++) {
    resetInterface(cartControllers[i]);
    cartControllers[i]->resetPose();
    cartControllers[i]->resetFt();
  }
  // ROS_INFO_STREAM("Restart!");
  RCLCPP_INFO_STREAM(this->get_logger(), "Restart!");
  // return true;
}

void Controller::setPriority(int i) {
  multimyik_solver_ptr->resetRobotWeight();  // make all robot priority equal.
  multimyik_solver_ptr->setRobotWeight(i, 100.);
}

void Controller::setLowPriority(int i) {
  multimyik_solver_ptr->resetRobotWeight();  // make all robot priority equal.
  // for (int j = 0; j++; j < nRobot)
  //   if (j != i)
  //     multimyik_solver_ptr->setRobotWeight(j, 100.);
  multimyik_solver_ptr->setRobotWeight(i, 0.1);
}

void Controller::setPriority(std::vector<int> idx) {
  multimyik_solver_ptr->resetRobotWeight();  // make all robot priority equal.

  double gain = pow(10.0, idx.size() - 1);
  for (auto& i : idx) {
    multimyik_solver_ptr->setRobotWeight(i, 100. * gain);
    gain *= 0.1;
  }
}

void Controller::setPriority(PriorityType priority) {
  std::vector<int> priorityInd;
  if (priority == PriorityType::Automation)
    priorityInd = autoInd;
  else
    priorityInd = manualInd;

  multimyik_solver_ptr->resetRobotWeight();  // make all robot priority equal.
  for (auto& ind : priorityInd)
    multimyik_solver_ptr->setRobotWeight(ind, 100.);
}

void Controller::setHightLowPriority(int high, int low) {
  multimyik_solver_ptr->resetRobotWeight();  // make all robot priority equal.
  multimyik_solver_ptr->setRobotWeight(high, 100.);
  multimyik_solver_ptr->setRobotWeight(low, 0.1);
}

void Controller::starting() {
  for (int i = 0; i < nRobot; i++) {
    cartControllers[i]->starting(this->get_clock()->now());
    exec.add_node(nodes[i]);
  }
  exec.add_node(this->node);

  // rclcpp::Duration(3.0).sleep();
  // rclcpp::sleep_for(3s);

  for (int i = 0; i < nRobot; i++)
    initInterface(cartControllers[i]);

  std::vector<KDL::JntArray> q_rest(nRobot);
  for (int i = 0; i < nRobot; i++)
    q_rest[i] = cartControllers[i]->getqRest();
  multimyik_solver_ptr->setqRest(q_rest);

  // cartControllers[i]->enableOperation();
  // }
  // ROS_INFO_STREAM("Controller started!");
  RCLCPP_INFO_STREAM(this->get_logger(), "Controller started!");
  this->t0 = this->get_clock()->now();
}

void Controller::stopping() {
  for (int i = 0; i < nRobot; i++)                           // {
    cartControllers[i]->stopping(this->get_clock()->now());  // TODO: Make sure that this works correctly.
  // cartControllers[i]->enableOperation();
  // }

  RCLCPP_INFO_STREAM(this->get_logger(), "Controller stopped!");

  rclcpp::shutdown();
}

void Controller::publishState(const rclcpp::Time& time, const std::vector<KDL::Frame> curPose, const std::vector<KDL::Twist> curVel, const std::vector<KDL::Frame> desPose,
                              const std::vector<KDL::Twist> desVel) {
  static rclcpp::Time prev = time;
  if ((time - prev).nanoseconds() * 1.0e-9 > 0.05) {
    for (int i = 0; i < nRobot; i++) {
      cartControllers[i]->publishDesEffPoseVel(desPose[i], desVel[i]);
      cartControllers[i]->publishCurEffPoseVel(curPose[i], curVel[i]);
    }
    prev = time;
  }
}

void Controller::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  static std::vector<KDL::JntArray> q_des(nRobot), dq_des(nRobot), q_cur(nRobot), dq_cur(nRobot);
  std::vector<KDL::Frame> curPose(nRobot);
  std::vector<KDL::Twist> curVel(nRobot);

  for (int i = 0; i < nRobot; i++)
    cartControllers[i]->getState(q_cur[i], dq_cur[i], curPose[i], curVel[i]);

  this->publishState(time, curPose, curVel, desPose, desVel);

  std::vector<KDL::JntArray> q_rest(nRobot);
  for (int i = 0; i < nRobot; i++)
    q_rest[i] = cartControllers[i]->getqRest();
  multimyik_solver_ptr->setqRest(q_rest);

  if (controller == ControllerType::Velocity) {
    int rc = multimyik_solver_ptr->CartToJntVel_qp(q_cur, desPose, desVel, dq_des, dt);

    if (rc < 0) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Failed to solve IK within dt. Skip this control loop");
      return;
    }

    // low pass filter
    for (int i = 0; i < nRobot; i++) {
      cartControllers[i]->filterJnt(dq_des[i]);
      if (q_des[i].data.rows() != dq_des[i].data.rows())
        q_des[i].data = cartControllers[i]->getqRest().data;
      q_des[i].data += dq_des[i].data * dt;
    }

    for (int i = 0; i < nRobot; i++) {
      if ((time - prev_time[i]).nanoseconds() * 1.0e-9 < 1.0 / cartControllers[i]->freq - 1.0 / this->freq)
        continue;

      prev_time[i] = time;
      // std::cout << q_des[i].data.transpose() << std::endl;
      // std::cout << dq_des[i].data.transpose() << std::endl;
      cartControllers[i]->sendVelCmd(q_des[i], dq_des[i], 1.0 / cartControllers[i]->freq);
    }

  } else if (controller == ControllerType::Position) {
    int rc = multimyik_solver_ptr->CartToJnt(q_cur, desPose, q_des, dt);

    if (rc < 0) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Failed to solve IK within dt. Skip this control loop");
      return;
    }

    // low pass filter
    for (int i = 0; i < nRobot; i++)
      cartControllers[i]->filterJnt(q_des[i]);

    for (int i = 0; i < nRobot; i++)
      cartControllers[i]->sendPosCmd(q_des[i], dq_des[i], dt);  // TODO: update dq

  } else
    RCLCPP_WARN_STREAM(this->get_logger(), "not implemented");

  for (int i = 0; i < nRobot; i++)
    feedback(desPose[i], desVel[i], cartControllers[i]);
}

void Controller::updateDesired() {
  for (int i = 0; i < nRobot; i++) {
    cartControllers[i]->updateCurState();

    tf2::transformEigenToKDL(cartControllers[i]->getT_init(), desPose[i]);
    desVel[i] = KDL::Twist();

    updateTargetPose(desPose[i], desVel[i], cartControllers[i]);

    // applyBaseControl(desPose[i], desVel[i], cartControllers[i]);

    cartControllers[i]->setDesired(desPose[i], desVel[i]);
  }

  preInterfaceProcess(interfaces);
}

void Controller::controlLoop() {
  rclcpp::Time t = this->get_clock()->now();
  rclcpp::Duration dur(0, dt * 1.0e9);
  // begin = std::chrono::high_resolution_clock::now();

  if (!std::all_of(cartControllers.begin(), cartControllers.end(), [](auto& c) { return c->isInitialized(); }))
    return;

  this->updateDesired();

  if (IKmode == IKMode::Order) {
    for (int i = 0; i < nRobot; i++)
      cartControllers[i]->update(t, dur);
  } else if (IKmode == IKMode::Parallel) {  // parallel IK(multithreading)
    std::vector<std::unique_ptr<std::thread>> workers(nRobot);
    sched_param sch;
    sch.sched_priority = 1;
    for (int i = 0; i < nRobot; i++) {
      auto c = cartControllers[i];
      workers[i] = std::make_unique<std::thread>([c, t, dur]() { c->update(t, dur); });
      // pthread_setschedparam(workers[i]->native_handle(), SCHED_FIFO, &sch);
    }
    std::for_each(workers.begin(), workers.end(), [](std::unique_ptr<std::thread>& th) { th->join(); });
  } else if (IKmode == IKMode::Concatenated)
    this->update(t, dur);

  // std::chrono::microseconds t = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - begin);
  // ROS_INFO_STREAM("IK time: " << t.count() * 1.0e-3 << "[ms]");
}

void Controller::control() {
  this->node = this->shared_from_this();

  std::vector<std::string> robots, hw_configs;
  if (!this->getRosParams(robots, hw_configs)) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Failed to get the initial parameters. Shutting down...");
    rclcpp::shutdown();
  }

  this->initMenbers(robots, hw_configs);

  this->starting();

  exec.spin();
}
