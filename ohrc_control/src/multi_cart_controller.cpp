#include "ohrc_control/multi_cart_controller.hpp"

MultiCartController::MultiCartController() {
  std::vector<std::string> robots;
  if (!getInitParam(robots))
    ros::shutdown();

  dt = 1.0 / freq;
  nRobot = robots.size();

  prev_time.resize(nRobot, ros::Time::now());

  cartControllers.resize(nRobot);
  for (int i = 0; i < nRobot; i++)
    // cartControllers[i].reset(new CartController(robots[i], root_frame, i));
    cartControllers[i].reset(new CartController(robots[i], hw_configs[i], root_frame, i));

  std::vector<std::string> base_link(nRobot), tip_link(nRobot);  // TODO: base_links for all robot shoud be same
  std::vector<Affine3d> T_base_root(nRobot);
  std::vector<std::shared_ptr<MyIK::MyIK>> myik_ptr(nRobot);
  for (int i = 0; i < nRobot; i++)
    cartControllers[i]->getInfo(base_link[i], tip_link[i], T_base_root[i], myik_ptr[i]);

  multimyik_solver_ptr.reset(new MyIK::MyIK(base_link, tip_link, T_base_root, myik_ptr));

  service = nh.advertiseService("/reset", &MultiCartController::resetService, this);

  // TODO: condifure this priority setting

  for (int i = 0; i < nRobot; i = i + 2)
    manualInd.push_back(i);

  for (int i = 1; i < nRobot; i = i + 2)
    autoInd.push_back(i);

  // std::cout << magic_enum::enum_name(priority) << std::endl;

  setPriority(priority);

  desPose.resize(nRobot);
  desVel.resize(nRobot);

  interfaces.resize(nRobot);

  admittanceControllers.resize(nRobot);
  for (int i = 0; i < nRobot; i++)
    admittanceControllers[i] = std::make_shared<AdmittanceController>(cartControllers[i]);
}

bool MultiCartController::getInitParam(std::vector<std::string>& robots) {
  ros::NodeHandle n("~");

  XmlRpc::XmlRpcValue my_list;
  if (!n.getParam("follower_list", my_list)) {
    ROS_FATAL_STREAM("Failed to get the hardware config list");
    return false;
  }

  for (auto itr = my_list.begin(); itr != my_list.end(); ++itr) {
    // std::cout << itr->first << ":" << itr->second << std::endl;
    if (my_list[itr->first].getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < my_list[itr->first].size(); ++i) {
        // std::cout << itr->first << ":" << my_list[itr->first][i] << std::endl;
        hw_configs.push_back(itr->first);
        robots.push_back(my_list[itr->first][i]);
      }
    }
  }

  // std::cout << my_list.size() << std::endl;

  // std::map<std::string, std::string> robots_map;
  // if (!n.getParam("follower_list", robots_map) || robots_map.empty()) {
  //   ROS_FATAL_STREAM("Failed to get the follower robot list");
  //   return false;
  // }

  for (int i = 0; i < robots.size(); i++)
    ROS_INFO_STREAM("Robot " << i << ": " << robots[i] << " - " << hw_configs[i]);

  if (!n.getParam("root_frame", root_frame)) {
    ROS_FATAL_STREAM("Failed to get the root frame of the robot system");
    return false;
  }

  if (!n.getParam("control_freq", freq)) {
    ROS_FATAL_STREAM("Failed to get the control_freq of the robot system");
    return false;
  }

  std::string controller_str;
  if (!n.param("controller", controller_str, std::string("Velocity")))
    ROS_WARN_STREAM("Controller is not choisen {Position, Velocity, Torque}: Default Velocity");
  else
    ROS_INFO_STREAM("Controller: " << controller_str);

  controller = magic_enum::enum_cast<ControllerType>(controller_str).value_or(ControllerType::None);
  if (controller == ControllerType::None) {
    ROS_FATAL("Controller type is not correctly choisen from {Position, Velocity, Torque}");
    return false;
  }

  // std::string publisher_str;
  // if (!n.param("publisher", publisher_str, std::string("Velocity")))
  //   ROS_WARN_STREAM("Publisher is not choisen {Position, Velocity, Torque, Trajectory, TrajectoryAction}: Default Velocity");
  // else
  //   ROS_INFO_STREAM("Publisher: " << publisher_str);

  // publisher = magic_enum::enum_cast<PublisherType>(publisher_str).value_or(PublisherType::None);
  // if (publisher == PublisherType::None) {
  //   ROS_FATAL("Publisher type is not correctly choisen from {Position, Velocity, Torque, Trajectory, TrajectoryAction}");
  //   return false;
  // }

  MFmode = this->getEnumParam("MF_mode", MFMode::None, "Individual", n);
  IKmode = this->getEnumParam("IK_mode", IKMode::None, "Concatenated", n);
  priority = this->getEnumParam("priority", PriorityType::None, "Manual", n);
  if (priority == PriorityType::Adaptation)
    n.param<std::string>("adaptation_option", adaptationOption_, "Default");

  if (!n.getParam("date", this->date)) {
    this->date = std_utility::getDatetimeStr();
    n.setParam("date", date);
  }

  return true;
}

bool MultiCartController::resetService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  for (int i = 0; i < nRobot; i++) {
    cartControllers[i]->resetPose();
    resetInterface(cartControllers[i]);
  }
  return true;
}

void MultiCartController::setPriority(int i) {
  multimyik_solver_ptr->resetRobotWeight();  // make all robot priority equal.
  multimyik_solver_ptr->setRobotWeight(i, 100.);
}

void MultiCartController::setLowPriority(int i) {
  multimyik_solver_ptr->resetRobotWeight();  // make all robot priority equal.
  // for (int j = 0; j++; j < nRobot)
  //   if (j != i)
  //     multimyik_solver_ptr->setRobotWeight(j, 100.);
  multimyik_solver_ptr->setRobotWeight(i, 0.1);
}

void MultiCartController::setPriority(std::vector<int> idx) {
  multimyik_solver_ptr->resetRobotWeight();  // make all robot priority equal.

  double gain = pow(10.0, idx.size() - 1);
  for (auto& i : idx) {
    multimyik_solver_ptr->setRobotWeight(i, 100. * gain);
    gain *= 0.1;
  }
}

void MultiCartController::setPriority(PriorityType priority) {
  std::vector<int> priorityInd;
  if (priority == PriorityType::Automation)
    priorityInd = autoInd;
  else
    priorityInd = manualInd;

  multimyik_solver_ptr->resetRobotWeight();  // make all robot priority equal.
  for (auto& ind : priorityInd)
    multimyik_solver_ptr->setRobotWeight(ind, 100.);
}

void MultiCartController::setHightLowPriority(int high, int low) {
  multimyik_solver_ptr->resetRobotWeight();  // make all robot priority equal.
  multimyik_solver_ptr->setRobotWeight(high, 100.);
  multimyik_solver_ptr->setRobotWeight(low, 0.1);
}

void MultiCartController::starting() {
  for (int i = 0; i < nRobot; i++)  // {
    cartControllers[i]->starting(ros::Time::now());

  ros::Duration(3.0).sleep();

  for (int i = 0; i < nRobot; i++)
    initInterface(cartControllers[i]);

  std::vector<KDL::JntArray> q_rest(nRobot);
  for (int i = 0; i < nRobot; i++)
    q_rest[i] = cartControllers[i]->getqRest();
  multimyik_solver_ptr->setqRest(q_rest);

  // cartControllers[i]->enableOperation();
  // }
  ROS_INFO_STREAM("Controller started!");
  this->t0 = ros::Time::now();
}

void MultiCartController::stopping() {
  for (int i = 0; i < nRobot; i++)                   // {
    cartControllers[i]->stopping(ros::Time::now());  // TODO: Make sure that this works correctly.
  // cartControllers[i]->enableOperation();
  // }
  ROS_INFO_STREAM("Controller stopped!");
  // this->t0 = ros::Time::now();
}

void MultiCartController::publishState(const ros::Time& time, const std::vector<KDL::Frame> curPose, const std::vector<KDL::Twist> curVel, const std::vector<KDL::Frame> desPose,
                                       const std::vector<KDL::Twist> desVel) {
  static ros::Time prev = time;
  if ((time - prev).toSec() > 0.05) {
    for (int i = 0; i < nRobot; i++) {
      cartControllers[i]->publishDesEffPoseVel(desPose[i], desVel[i]);
      cartControllers[i]->publishCurEffPoseVel(curPose[i], curVel[i]);
    }
    prev = time;
  }
}

void MultiCartController::update(const ros::Time& time, const ros::Duration& period) {
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
      ROS_WARN_STREAM("Failed to solve IK within dt. Skip this control loop");
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
      if ((time - prev_time[i]) < ros::Duration(1.0 / cartControllers[i]->freq - 1.0 / this->freq))
        continue;

      prev_time[i] = time;
      // std::cout << q_des[i].data.transpose() << std::endl;
      // std::cout << dq_des[i].data.transpose() << std::endl;
      cartControllers[i]->sendVelCmd(q_des[i], dq_des[i], 1.0 / cartControllers[i]->freq);
    }

  } else if (controller == ControllerType::Position) {
    int rc = multimyik_solver_ptr->CartToJnt(q_cur, desPose, q_des, dt);

    if (rc < 0) {
      ROS_WARN_STREAM("Failed to solve IK within dt. Skip this control loop");
      return;
    }

    // low pass filter
    for (int i = 0; i < nRobot; i++)
      cartControllers[i]->filterJnt(q_des[i]);

    for (int i = 0; i < nRobot; i++)
      cartControllers[i]->sendPosCmd(q_des[i], dq_des[i], dt);  // TODO: update dq

  } else
    ROS_WARN_STREAM("not implemented");

  for (int i = 0; i < nRobot; i++)
    feedback(desPose[i], desVel[i], cartControllers[i]);
}

void MultiCartController::updateDesired() {
  for (int i = 0; i < nRobot; i++) {
    cartControllers[i]->updateCurState();

    tf::transformEigenToKDL(cartControllers[i]->getT_init(), desPose[i]);
    desVel[i] = KDL::Twist();

    updateTargetPose(desPose[i], desVel[i], cartControllers[i]);

    if (enbaleAdmittanceControl)
      applyAdmittanceControl(desPose[i], desVel[i], cartControllers[i]);

    cartControllers[i]->setDesired(desPose[i], desVel[i]);
  }

  preInterfaceProcess(interfaces);
}

int MultiCartController::control() {
  this->starting();

  std::vector<double> ms(3, 0.0);
  std::chrono::high_resolution_clock::time_point begin;
  std::vector<std::unique_ptr<std::thread>> workers(nRobot);
  sched_param sch;
  sch.sched_priority = 1;

  double count = 0.0;

  ros::Rate r(freq);

  while (ros::ok()) {
    // begin = std::chrono::high_resolution_clock::now();

    if (!std::all_of(cartControllers.begin(), cartControllers.end(), [](auto& c) { return c->isInitialized(); }))
      continue;

    updateDesired();

    if (IKmode == IKMode::Order) {
      for (int i = 0; i < nRobot; i++)
        cartControllers[i]->update();
    } else if (IKmode == IKMode::Parallel) {  // parallel IK(multithreading)
      for (int i = 0; i < nRobot; i++) {
        CartController* c = cartControllers[i].get();
        workers[i].reset(new std::thread([c]() { c->update(); }));
        // pthread_setschedparam(workers[i]->native_handle(), SCHED_FIFO, &sch);
      }
      std::for_each(workers.begin(), workers.end(), [](std::unique_ptr<std::thread>& th) { th->join(); });
    } else if (IKmode == IKMode::Concatenated)
      this->update(ros::Time::now(), ros::Duration(dt));

    // std::chrono::microseconds t = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - begin);
    // ROS_INFO_STREAM("IK time: " << t.count() * 1.0e-3 << "[ms]");

    r.sleep();
  }

  this->stopping();

  return 1;
}
