#include "ohrc_control/multi_cart_controller.hpp"

MultiCartController::MultiCartController() {
  if (!getInitParam())
    ros::shutdown();

  dt = 1.0 / freq;
  nRobot = robots.size();

  cartControllers.resize(nRobot);
  for (int i = 0; i < nRobot; i++)
    cartControllers[i].reset(new CartController(robots[i], root_frame, i));

  std::vector<std::string> base_link(nRobot), tip_link(nRobot), URDF_param(nRobot);  // TODO: base_links for all robot shoud be same
  std::vector<Affine3d> T_base_root(nRobot);
  for (int i = 0; i < nRobot; i++)
    cartControllers[i]->getInfo(base_link[i], tip_link[i], URDF_param[i], T_base_root[i]);

  multimyik_solver_ptr.reset(new MyIK::MultiMyIK(base_link, tip_link, URDF_param, T_base_root));

  // TODO: condifure this priority setting
  controller = ControllerType::Velocity;
  for (int i = 0; i < nRobot; i = i + 2)
    manualInd.push_back(i);

  for (int i = 1; i < nRobot; i = i + 2)
    autoInd.push_back(i);

  setPriority(priority);
}

bool MultiCartController::getInitParam() {
  ros::NodeHandle n("~");

  if (!n.getParam("follower_list", robots) || !robots.size()) {
    ROS_FATAL_STREAM("Failed to get the follower robot list");
    return false;
  }

  if (!n.getParam("root_frame", root_frame)) {
    ROS_FATAL_STREAM("Failed to get the root frame of the robot system");
    return false;
  }

  if (!n.getParam("control_freq", freq)) {
    ROS_FATAL_STREAM("Failed to get the control_freq of the robot system");
    return false;
  }

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

void MultiCartController::starting() {
  for (int i = 0; i < nRobot; i++) {
    cartControllers[i]->starting(ros::Time::now());
    cartControllers[i]->enableOperation();
  }
  ROS_INFO_STREAM("Controller started!");
  this->t0 = ros::Time::now();
}

void MultiCartController::update(const ros::Time& time, const ros::Duration& period) {
  std::vector<KDL::JntArray> dq_des(nRobot), q_cur(nRobot), dq_cur(nRobot);
  std::vector<KDL::Frame> des_eff_pose(nRobot);
  std::vector<KDL::Twist> des_eff_vel(nRobot);

  for (int i = 0; i < nRobot; i++) {
    cartControllers[i]->getState(q_cur[i], dq_cur[i]);
    tf::transformEigenToKDL(cartControllers[i]->getT_init(), des_eff_pose[i]);
  }

  if (MFmode == MFMode::Individual) {
    for (auto& ind : manualInd)
      updateManualTargetPose(des_eff_pose[ind], des_eff_vel[ind], cartControllers[ind].get());
    for (auto& ind : autoInd)
      updateAutoTargetPose(des_eff_pose[ind], des_eff_vel[ind], cartControllers[ind].get());

  } else if (MFmode == MFMode::Parallel) {
    for (auto& ind : manualInd)
      updateManualTargetPose(des_eff_pose[ind], des_eff_vel[ind], cartControllers[ind].get());
    for (auto& ind : autoInd) {
      des_eff_pose[ind] = des_eff_pose[manualInd[0]];
      des_eff_vel[ind] = des_eff_vel[manualInd[0]];
      updateAutoTargetPose(des_eff_pose[ind], des_eff_vel[ind], cartControllers[ind].get());
    }
  } else if (MFmode == MFMode::Cooperation) {
  }

  int rc = multimyik_solver_ptr->CartToJntVel_qp(q_cur, des_eff_pose, des_eff_vel, dq_des, dt);

  if (rc < 0) {
    ROS_WARN_STREAM("Failed to solve IK. Skip this control loop");
    return;
  }

  // low pass filter
  for (int i = 0; i < nRobot; i++)
    cartControllers[i]->filterJnt(dq_des[i]);

  static ros::Time prev = time;
  if ((time - prev).toSec() > 0.05) {
    for (int i = 0; i < nRobot; i++)
      cartControllers[i]->publishDesEffPoseVel(des_eff_pose[i], des_eff_vel[i]);
    prev = time;
  }
  for (int i = 0; i < nRobot; i++) {
    std_msgs::Float64MultiArray cmd;
    cmd.data = std::vector<double>(dq_des[i].data.data(), dq_des[i].data.data() + dq_des[i].data.rows() * dq_des[i].data.cols());
    cartControllers[i]->jntVelCmdPublisher.publish(cmd);
  }

  for (auto& ind : manualInd) {
    // KDL::JntArray q_des;
    // q_des.data = q_cur[ind].data + dq_des[ind].data * dt;
    // feedbackJnt(q_cur[ind], q_des, cartControllers[ind].get());
    q_cur[ind].data += dq_des[ind].data * dt;
    KDL::Frame p;
    cartControllers[ind]->JntToCart(q_cur[ind], p);

    Affine3d T_cur, T_des;
    tf::transformKDLToEigen(p, T_cur);
    tf::transformKDLToEigen(des_eff_pose[ind], T_des);
    feedbackCart(T_cur, T_des, cartControllers[ind].get());
  }
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
    if (IKmode == IKMode::Order) {
      for (int i = 0; i < robots.size(); i++)
        cartControllers[i]->update();
    } else if (IKmode == IKMode::Parallel) {  // parallel IK(multithreading)
      for (int i = 0; i < robots.size(); i++) {
        CartController* c = cartControllers[i].get();
        workers[i].reset(new std::thread([c]() { c->update(); }));
        // pthread_setschedparam(workers[i]->native_handle(), SCHED_FIFO, &sch);
      }
      std::for_each(workers.begin(), workers.end(), [](std::unique_ptr<std::thread>& th) { th->join(); });
    } else if (IKmode == IKMode::Concatenated)
      this->update(ros::Time::now(), ros::Duration(dt));

    r.sleep();
  }

  return 1;
}
