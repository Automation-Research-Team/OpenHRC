#ifndef CART_CONTROLLER_HPP
#define CART_CONTROLLER_HPP

#include <geometry_msgs/msg/transform_stamped.hpp>
// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>
#include <boost/date_time.hpp>
#include <csignal>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <mutex>
#include <trac_ik/trac_ik.hpp>
#include <vector>

#include "ohrc_control/my_ik.hpp"

class ForwardingController {
  static void signal_handler(int signum);

protected:
  enum SolverType { Trac_IK, KDL, MyIK } solver;
  enum ControllerType { Position, Velocity, Torque } controller;
  enum RobotType { Follower, Leader };
  // ros::NodeHandle nh;
  std::vector<ros::Publisher> jntPosCmdPublisher, jntVelCmdPublisher;
  std::vector<ros::Subscriber> jntStateSubscriber;

  ros::AsyncSpinner spinner;

  std::mutex mtx;
  std::vector<bool*> subFlagPtrs;
  bool flagJntState = false, flagIfJntState = false, flagEffPose = false;

  int num_samples;
  std::string chain_start, chain_end, urdf_param;
  std::vector<std::string> robot_ns;
  double timeout;
  const double eps = 1e-5;

  unsigned int nJnt;         // number of robot joint
  const unsigned int m = 6;  // number of target DoF (usually 6)

  const double freq = 500.0;

  KDL::Frame _des_eff_pose;
  KDL::Frame _current_eff_pose;
  KDL::Twist _des_eff_vel;

  KDL::JntArray _q_cur, _q_cur_if, _dq_cur, _dq_cur_if, _q_des;

  Affine3d T_init;

  tf2_ros::TransformBroadcaster br;

  // KDL
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> vik_solver_ptr;   // PseudoInverse vel solver
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> kdl_solver_ptr;  // Joint Limit Solver

  // TRAC-IK
  std::unique_ptr<TRAC_IK::TRAC_IK> tracik_solver_ptr;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_ptr;
  KDL::Chain chain;
  std::vector<KDL::Segment> chain_segs;

  // MyIK
  std::unique_ptr<MyIK::MyIK> myik_solver_ptr;

  void cbJntState(const sensor_msgs::JointState::ConstPtr& msg);
  void cbIfJntState(const sensor_msgs::JointState::ConstPtr& msg);
  void getVelocity(const KDL::Frame& frame, const KDL::Frame& prev_frame, const double& dt, KDL::Twist& twist);

  void initDesWithJnt(const KDL::JntArray& q_init);
  virtual void initWithJnt(const KDL::JntArray& q_init){};
  virtual void getDesEffPoseVel(const double& dt, const KDL::JntArray& q_cur, KDL::Frame& des_eff_pose, KDL::Twist& des_eff_vel);
  // void publishDesEffPoseVel(const KDL::Frame& des_eff_pose, const KDL::Twist& des_eff_vel);

  int moveInitJntPos(const KDL::JntArray& q_cur);
  bool getJntState(const sensor_msgs::JointState::ConstPtr& msg, KDL::JntArray& q_cur, KDL::JntArray& dq_cur, RobotType type, bool& isFirst, bool& initialized, bool& isFirst2, KDL::JntArray& q_des,
                   KDL::JntArray& q_init);
  // int moveInitPos();

public:
  ForwardingController();
  ~ForwardingController();
  int control();
};

#endif  // CART_CONTROLLER_HPP