#ifndef MULTI_CART_COTNROLLER_HPP
#define MULTI_CART_COTNROLLER_HPP

#include <numeric>
#include <thread>

#include "ohrc_control/cart_controller.hpp"
#include "ohrc_control/interface.hpp"
#include "ohrc_control/multi_my_ik.hpp"
#include "ohrc_control/ohrc_control.hpp"

using namespace ohrc_control;

class MultiCartController {
  bool getInitParam();
  void updateDesired();
  std::vector<KDL::Frame> desPose;
  std::vector<KDL::Twist> desVel;

  ros::ServiceServer service;
  bool resetService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

protected:
  ros::NodeHandle nh;

  enum class MFMode { None, Individual, Parallel, Cooperation } MFmode;
  enum class IKMode { None, Concatenated, Order, Parallel } IKmode;

  std::vector<std::shared_ptr<CartController>> cartControllers;
  std::vector<std::string> robots;
  std::vector<std::string> hw_configs;
  int nRobot = 0;

  std::string root_frame;
  double freq = 500.0;
  double dt = 0.002;
  ros::Time t0;
  std::string date;

  // MyIK
  std::unique_ptr<MyIK::MultiMyIK> multimyik_solver_ptr;

  ControllerType controller;
  PublisherType publisher;

  std::vector<int> manualInd, autoInd;

  enum class PriorityType { Manual, Automation, Adaptation, None } priority;
  bool adaptation = false;
  void setPriority(PriorityType priority);
  void setPriority(std::vector<int> idx);
  void setPriority(int i);
  void setLowPriority(int i);
  void setHightLowPriority(int high, int low);

  // enum class AdaptationOption { Default, None } adaptationOption;
  std::string adaptationOption_;

  virtual void runLoopEnd(){};

  std::vector<std::shared_ptr<Interface>> interfaces;
  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist, std::shared_ptr<CartController> controller) {
    interfaces[controller->getIndex()]->updateTargetPose(pose, twist);
  }

  void initInterface(std::shared_ptr<CartController> controller) {
    interfaces[controller->getIndex()]->initInterface();
  }

  void resetInterface(std::shared_ptr<CartController> controller) {
    interfaces[controller->getIndex()]->resetInterface();
  }

  void feedback(KDL::Frame& pose, KDL::Twist& twist, std::shared_ptr<CartController> controller) {
    interfaces[controller->getIndex()]->feedback(pose, twist);
  }

  virtual void preInterfaceProcess(std::vector<std::shared_ptr<Interface>> interfaces){};

  virtual void updateManualTargetPose(KDL::Frame& pose, KDL::Twist& twist, std::shared_ptr<CartController> controller) {
    updateTargetPose(pose, twist, controller);
  };
  virtual void updateAutoTargetPose(KDL::Frame& pose, KDL::Twist& twist, std::shared_ptr<CartController> controller) {
    updateTargetPose(pose, twist, controller);
  };
  // virtual void feedbackJnt(const KDL::JntArray& q_cur, const KDL::JntArray& q_des, std::shared_ptr<CartController> controller){};
  // virtual void feedbackCart(const Affine3d& T_cur, const Affine3d& T_des, std::shared_ptr<CartController> controller){};

  template <typename T>
  inline T getEnumParam(const std::string& key, T none, const std::string default_str, ros::NodeHandle n) {
    std::string s;
    if (!n.getParam(key, s)) {
      ROS_INFO_STREAM("Failed to get " << key << ", so" << default_str << "is automatically selected");
      s = default_str;
    }

    T mode = magic_enum::enum_cast<T>(s).value_or(none);
    if (mode == none) {
      ROS_FATAL_STREAM(key << " is configured as [" << s << "] and not correctly configured.");
      ros::shutdown();
    } else
      ROS_INFO_STREAM("Operation mode: " << s);

    return mode;
  }

public:
  MultiCartController();
  int control();
  virtual void starting();
  void stopping();
  void update(const ros::Time& time, const ros::Duration& period);
};

#endif  // MULTI_CART_COTNROLLER_HPP
/*
template <class T>
class MultiCartController {
  bool getInitParam();
  void updateDesired();
  std::vector<KDL::Frame> desPose;
  std::vector<KDL::Twist> desVel;

  ros::ServiceServer service;
  bool resetService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

protected:
  ros::NodeHandle nh;

  std::vector<std::shared_ptr<T>> interfaces_;

  enum class MFMode { None, Individual, Parallel, Cooperation } MFmode;
  enum class IKMode { None, Concatenated, Order, Parallel } IKmode;

  std::vector<std::shared_ptr<CartController>> cartControllers;
  std::vector<std::string> robots;
  int nRobot = 0;

  std::string root_frame;
  double freq = 500.0;
  double dt = 0.002;
  ros::Time t0;
  std::string date;

  // MyIK
  std::unique_ptr<MyIK::MultiMyIK> multimyik_solver_ptr;

  ControllerType controller;
  PublisherType publisher;

  std::vector<int> manualInd, autoInd;

  enum class PriorityType { Manual, Automation, Adaptation, None } priority;
  bool adaptation = false;
  void setPriority(PriorityType priority);

  // enum class AdaptationOption { Default, None } adaptationOption;
  std::string adaptationOption_;

  virtual void runLoopEnd(){};
  virtual void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist, std::shared_ptr<CartController> controller){};
  virtual void updateManualTargetPose(KDL::Frame& pose, KDL::Twist& twist, std::shared_ptr<CartController> controller) {
    updateTargetPose(pose, twist, controller);
  };
  virtual void updateAutoTargetPose(KDL::Frame& pose, KDL::Twist& twist, std::shared_ptr<CartController> controller) {
    updateTargetPose(pose, twist, controller);
  };
  // virtual void feedbackJnt(const KDL::JntArray& q_cur, const KDL::JntArray& q_des, std::shared_ptr<CartController> controller){};
  // virtual void feedbackCart(const Affine3d& T_cur, const Affine3d& T_des, std::shared_ptr<CartController> controller){};

  virtual void resetInterface(std::shared_ptr<CartController> controller){};

  template <typename U>
  inline U getEnumParam(const std::string& key, U none, const std::string default_str, ros::NodeHandle n) {
    std::string s;
    if (!n.getParam(key, s)) {
      ROS_INFO_STREAM("Failed to get " << key << ", so" << default_str << "is automatically selected");
      s = default_str;
    }

    U mode = magic_enum::enum_cast<U>(s).value_or(none);
    if (mode == none) {
      ROS_FATAL_STREAM(key << " is configured as [" << s << "] and not correctly configured.");
      ros::shutdown();
    } else
      ROS_INFO_STREAM("Operation mode: " << s);

    return mode;
  }

  void setInterface(std::vector<std::shared_ptr<T>> interfaces) {
    this->interfaces_ = interfaces;
  }

public:
  MultiCartController() {
    init();
  };

  void init();
  int control();
  virtual void starting();
  void stopping();
  void update(const ros::Time& time, const ros::Duration& period);
};

// #include "ohrc_control/multi_cart_controller.hpp"

template <class T>
void MultiCartController<T>::init() {
  if (!getInitParam())
    ros::shutdown();

  dt = 1.0 / freq;
  nRobot = robots.size();

  cartControllers.resize(nRobot);
  for (int i = 0; i < nRobot; i++)
    cartControllers[i].reset(new CartController(robots[i], root_frame, i));

  std::vector<std::string> base_link(nRobot), tip_link(nRobot), URDF_param(nRobot);  // TODO: base_links for all robot shoud be same
  std::vector<Affine3d> T_base_root(nRobot);
  std::vector<std::shared_ptr<MyIK::MyIK>> myik_ptr(nRobot);
  for (int i = 0; i < nRobot; i++)
    cartControllers[i]->getInfo(base_link[i], tip_link[i], URDF_param[i], T_base_root[i], myik_ptr[i]);

  multimyik_solver_ptr.reset(new MyIK::MultiMyIK(base_link, tip_link, URDF_param, T_base_root, myik_ptr));

  service = nh.advertiseService("/reset", &MultiCartController::resetService, this);

  // TODO: condifure this priority setting

  for (int i = 0; i < nRobot; i = i + 2)
    manualInd.push_back(i);

  for (int i = 1; i < nRobot; i = i + 2)
    autoInd.push_back(i);

  std::cout << magic_enum::enum_name(priority) << std::endl;

  setPriority(priority);

  desPose.resize(nRobot);
  desVel.resize(nRobot);
}
template <class T>
bool MultiCartController<T>::getInitParam() {
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

  std::string publisher_str;
  if (!n.param("publisher", publisher_str, std::string("Velocity")))
    ROS_WARN_STREAM("Publisher is not choisen {Position, Velocity, Torque, Trajectory, TrajectoryAction}: Default Velocity");
  else
    ROS_INFO_STREAM("Publisher: " << publisher_str);

  publisher = magic_enum::enum_cast<PublisherType>(publisher_str).value_or(PublisherType::None);
  if (publisher == PublisherType::None) {
    ROS_FATAL("Publisher type is not correctly choisen from {Position, Velocity, Torque, Trajectory, TrajectoryAction}");
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
template <class T>
bool MultiCartController<T>::resetService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  for (int i = 0; i < nRobot; i++) {
    cartControllers[i]->resetPose();
    resetInterface(cartControllers[i]);
  }
  return true;
}
template <class T>
void MultiCartController<T>::setPriority(PriorityType priority) {
  std::vector<int> priorityInd;
  if (priority == PriorityType::Automation)
    priorityInd = autoInd;
  else
    priorityInd = manualInd;

  multimyik_solver_ptr->resetRobotWeight();  // make all robot priority equal.
  for (auto& ind : priorityInd)
    multimyik_solver_ptr->setRobotWeight(ind, 100.);
}
template <class T>
void MultiCartController<T>::starting() {
  for (int i = 0; i < nRobot; i++)  // {
    cartControllers[i]->starting(ros::Time::now());
  // cartControllers[i]->enableOperation();
  // }
  ROS_INFO_STREAM("Controller started!");
  this->t0 = ros::Time::now();
}
template <class T>
void MultiCartController<T>::stopping() {
  for (int i = 0; i < nRobot; i++)                   // {
    cartControllers[i]->stopping(ros::Time::now());  // TODO: Make sure that this works correctly.
  // cartControllers[i]->enableOperation();
  // }
  ROS_INFO_STREAM("Controller stopped!");
  // this->t0 = ros::Time::now();
}
template <class T>
void MultiCartController<T>::update(const ros::Time& time, const ros::Duration& period) {
  std::vector<KDL::JntArray> q_des(nRobot), dq_des(nRobot), q_cur(nRobot), dq_cur(nRobot);
  for (int i = 0; i < nRobot; i++) {
    cartControllers[i]->updateCurState();
    cartControllers[i]->getState(q_cur[i], dq_cur[i]);
  }
  static ros::Time prev = time;
  if ((time - prev).toSec() > 0.05) {
    for (int i = 0; i < nRobot; i++)
      cartControllers[i]->publishDesEffPoseVel(desPose[i], desVel[i]);
    prev = time;
  }

  if (controller == ControllerType::Velocity) {
    int rc = multimyik_solver_ptr->CartToJntVel_qp(q_cur, desPose, desVel, dq_des, dt);

    if (rc < 0) {
      ROS_WARN_STREAM("Failed to solve IK within dt. Skip this control loop");
      return;
    }

    // low pass filter
    for (int i = 0; i < nRobot; i++)
      cartControllers[i]->filterJnt(dq_des[i]);

    for (int i = 0; i < nRobot; i++) {
      std_msgs::Float64MultiArray cmd;
      cmd.data = std::vector<double>(dq_des[i].data.data(), dq_des[i].data.data() + dq_des[i].data.rows() * dq_des[i].data.cols());
      cartControllers[i]->jntCmdPublisher.publish(cmd);
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

    for (int i = 0; i < nRobot; i++) {
      if (publisher == PublisherType::Position) {
        std_msgs::Float64MultiArray cmd;
        cmd.data = std::vector<double>(q_des[i].data.data(), q_des[i].data.data() + q_des[i].data.rows() * q_des[i].data.cols());
        cartControllers[i]->jntCmdPublisher.publish(cmd);
      } else {
        trajectory_msgs::JointTrajectory cmd_trj;
        cmd_trj.points.resize(1);
        cmd_trj.points[0].time_from_start = ros::Duration(dt);
        for (int j = 0; j < cartControllers[i]->getNJnt(); j++) {
          cmd_trj.joint_names.push_back(cartControllers[i]->getNameJnt()[j]);
          // cmd_trj.joint_names.push_back(chain_segs[i].getJoint().getName());
          cmd_trj.points[0].positions.push_back(q_des[i].data[j]);
        }

        if (publisher == PublisherType::Trajectory) {
          cartControllers[i]->jntCmdPublisher.publish(cmd_trj);
        } else if (publisher == PublisherType::TrajectoryAction) {
          control_msgs::FollowJointTrajectoryActionGoal cmd_trjAction;
          cmd_trjAction.goal.trajectory = cmd_trj;
          cartControllers[i]->jntCmdPublisher.publish(cmd_trjAction);
        }
      }
    }

  } else
    ROS_WARN_STREAM("not implemented");

  // for (auto& ind : manualInd) {
  //   // KDL::JntArray q_des;
  //   // q_des.data = q_cur[ind].data + dq_des[ind].data * dt;
  //   // feedbackJnt(q_cur[ind], q_des, cartControllers[ind].get());
  //   if (controller == ControllerType::Velocity)
  //     q_cur[ind].data += dq_des[ind].data * dt;
  //   else
  //     q_cur[ind].data = q_des[ind].data;
  //   KDL::Frame p;
  //   cartControllers[ind]->JntToCart(q_cur[ind], p);

  //   Affine3d T_cur, T_des;
  //   tf::transformKDLToEigen(p, T_cur);
  //   tf::transformKDLToEigen(desPose[ind], T_des);
  //   // cartControllers[ind]->feedbackCart(T_cur, T_des);
  // }
}
template <class T>
void MultiCartController<T>::updateDesired() {
  for (int i = 0; i < nRobot; i++) {
    tf::transformEigenToKDL(cartControllers[i]->getT_init(), desPose[i]);
    desVel[i] = KDL::Twist();
  }

  if (MFmode == MFMode::Individual) {
    for (auto& ind : manualInd)
      updateManualTargetPose(desPose[ind], desVel[ind], cartControllers[ind]);
    for (auto& ind : autoInd)
      updateAutoTargetPose(desPose[ind], desVel[ind], cartControllers[ind]);

  } else if (MFmode == MFMode::Parallel) {
    for (auto& ind : manualInd)
      updateManualTargetPose(desPose[ind], desVel[ind], cartControllers[ind]);
    for (auto& ind : autoInd) {
      desPose[ind] = desPose[manualInd[0]];
      desVel[ind] = desVel[manualInd[0]];
      updateAutoTargetPose(desPose[ind], desVel[ind], cartControllers[ind]);
    }
  } else if (MFmode == MFMode::Cooperation) {
    ROS_ERROR_STREAM("This MFmode is not implemented");
  }

  for (int i = 0; i < robots.size(); i++)
    cartControllers[i]->setDesired(desPose[i], desVel[i]);
}
template <class T>
int MultiCartController<T>::control() {
  this->starting();

  std::vector<double> ms(3, 0.0);
  std::chrono::high_resolution_clock::time_point begin;
  std::vector<std::unique_ptr<std::thread>> workers(nRobot);
  sched_param sch;
  sch.sched_priority = 1;

  double count = 0.0;

  ros::Rate r(freq);

  while (ros::ok()) {
    if (!std::all_of(cartControllers.begin(), cartControllers.end(), [](auto& c) { return c->isInitialized(); }))
      continue;

    updateDesired();

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

  this->stopping();

  return 1;
}

#endif  // MULTI_CART_COTNROLLER_HPP
*/
