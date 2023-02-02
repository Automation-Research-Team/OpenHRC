#ifndef CART_CONTROLLER_HPP
#define CART_CONTROLLER_HPP

#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Geometry>
#include <boost/date_time.hpp>
#include <csignal>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <mutex>
#include <trac_ik/trac_ik.hpp>
#include <vector>

#include "magic_enum.hpp"
#include "ohrc_common/filter_utility/butterworth.h"
#include "ohrc_control/arm_marker.hpp"
#include "ohrc_control/my_ik.hpp"
#include "ohrc_control/ohrc_control.hpp"
#include "ohrc_msgs/StateStamped.h"
#include "std_srvs/Empty.h"

// TODO: Add namespace "Controllers"?

using namespace ohrc_control;

class CartController {
  static void signal_handler(int signum);
  void init(std::string robot);
  ros::AsyncSpinner spinner;

  KDL::Frame des_eff_pose, current_eff_pose;
  KDL::Twist des_eff_vel;
  KDL::JntArray q_cur, dq_cur;
  std_msgs::Float64MultiArray cmd;
  Matrix3d userManipU;
  int rc;

  struct s_cbJntState {
    bool isFirst = true;
    bool initialized = false;
  } s_cbJntState;

  struct s_moveInitPos {
    bool isFirst = true;
    KDL::JntArray q_des;
    KDL::JntArray q_init;
    ros::Time t_s;
    bool isSentTrj = false;
  } s_moveInitPos;

  std::vector<double> _q_init_expect;

  std::vector<double> initPose;
  bool getInitParam();

  std::string publisherTopicName;

  ros::ServiceServer service;
  bool resetService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool initialized = false;

  // KDL::JntArray dq_des;
  // KDL::JntArray q_des;

protected:
  SolverType solver;
  ControllerType controller;
  PublisherType publisher;
  ros::NodeHandle nh;

  ros::Subscriber jntStateSubscriber, userArmMarker, subForce;
  ros::TransportHints th = ros::TransportHints().tcpNoDelay(true);

  std::vector<bool*> subFlagPtrs;

  bool flagEffPose = false;

  Matrix3d _userManipU = Matrix3d::Identity();
  bool flagArmMarker = false;
  bool useManipOpt = false;

  int num_samples;

  double timeout;

  Affine3d T_init;
  Affine3d Tft_eff;

  unsigned int nJnt;         // number of robot joint
  const unsigned int m = 6;  // number of target DoF (usually 6)

  std::vector<std::string> nameJnt;

  KDL::Frame _des_eff_pose;
  KDL::Frame _current_eff_pose;
  KDL::Twist _des_eff_vel;

  geometry_msgs::WrenchStamped _force;

  bool _disable = true;

  KDL::JntArray _q_cur, _dq_cur;

  std::vector<butterworth> velFilter, jntFilter;

  TransformUtility trans;
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
  std::shared_ptr<MyIK::MyIK> myik_solver_ptr;

  std::string root_frame;
  std::string chain_start, chain_end, urdf_param, robot_ns = "";
  Affine3d T_base_root;

  void cbJntState(const sensor_msgs::JointState::ConstPtr& msg);
  void cbArmMarker(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void cbForce(const geometry_msgs::WrenchStamped::ConstPtr& msg);

  void initDesWithJnt(const KDL::JntArray& q_init);
  virtual void initWithJnt(const KDL::JntArray& q_init);
  virtual void getDesEffPoseVel(const double& dt, const KDL::JntArray& q_cur, const KDL::JntArray& dq_cur, KDL::Frame& des_eff_pose, KDL::Twist& des_eff_vel);
  void filterDesEffPoseVel(KDL::Frame& des_eff_pose, KDL::Twist& des_eff_vel);

  int moveInitPos(const KDL::JntArray& q_cur, const std::vector<std::string> nameJnt, std::vector<int> idxSegJnt);

  const int index = 0;
  // int moveInitPos();

  void resetFt();

  void sendPositionCmd(const VectorXd& q_des);
  void sendVelocityCmd(const VectorXd& dq_des);
  void sendVelocityCmd(const VectorXd& q_des, const VectorXd& dq_des, const KDL::JntArray& q_cur, const bool& lastLoop);
  void sendTrajectoryCmd(const VectorXd& q_des, const double& T);
  void sendTrajectoryCmd(const VectorXd& q_des, const VectorXd& dq_des, const double& T);
  void sendTrajectoryActionCmd(const VectorXd& q_des, const double& T);
  void sendTrajectoryActionCmd(const VectorXd& q_des, const VectorXd& dq_des, const double& T);
  void getTrajectoryCmd(const VectorXd& q_des, const double& T, trajectory_msgs::JointTrajectory& cmd_trj);
  void getTrajectoryCmd(const VectorXd& q_des, const VectorXd& dq_des, const double& T, trajectory_msgs::JointTrajectory& cmd_trj);

public:
  CartController();
  CartController(const std::string robot, const std::string root_frame);
  CartController(const std::string robot, const std::string root_frame, const int index);
  ~CartController();
  int control();

  void update();
  void update(const ros::Time& time, const ros::Duration& period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);

  void getIKInput(double dt, KDL::JntArray& q_cur, KDL::Frame& des_eff_pose, KDL::Twist& des_eff_vel);
  void getVelocity(const KDL::Frame& frame, const KDL::Frame& prev_frame, const double& dt, KDL::Twist& twist) const;
  void getState(KDL::JntArray& q_cur, KDL::JntArray& dq_cur) {
    std::lock_guard<std::mutex> lock(mtx);
    q_cur = this->_q_cur;
    dq_cur = this->_dq_cur;
  }

  void getState(KDL::JntArray& q_cur, KDL::JntArray& dq_cur, KDL::Frame& frame, KDL::Twist& twist) {
    getState(q_cur, dq_cur);

    JntToCart(q_cur, frame);
    JntVelToCartVel(q_cur, dq_cur, twist);
  }

  void getCartState(KDL::Frame& frame, KDL::Twist& twist) {
    KDL::JntArray q_cur;
    KDL::JntArray dq_cur;
    getState(q_cur, dq_cur, frame, twist);
  }

  void publishDesEffPoseVel(const KDL::Frame& des_eff_pose, const KDL::Twist& des_eff_vel);
  void getDesState(const KDL::Frame& cur_pose, const KDL::Twist& cur_vel, KDL::Frame& des_pose, KDL::Twist& des_vel);
  void publishState(const KDL::Frame& pose, const KDL::Twist& vel, ros::Publisher* publisher);
  void publishMarker(const KDL::JntArray q_cur);

  void filterJnt(KDL::JntArray& q);

  void enableOperation() {
    _disable = false;
  }

  void disableOperation() {
    _disable = true;
  }

  bool getOperationEnable() const {
    return !_disable;
  }
  void getInfo(std::string& chain_start, std::string& chain_end, std::string& urdf_param, Affine3d& T_base_root, std::shared_ptr<MyIK::MyIK>& myik) {
    chain_start = getChainStart();
    chain_end = this->chain_end;
    urdf_param = this->urdf_param;
    T_base_root = this->T_base_root;
    myik = this->myik_solver_ptr;
  }
  std::string getChainStart() const {
    return chain_start;
  }

  std::string getChainEnd() const {
    return chain_end;
  }

  void JntToCart(const KDL::JntArray& q_in, KDL::Frame& p_out) {
    myik_solver_ptr->JntToCart(q_in, p_out);
  }

  void JntVelToCartVel(const KDL::JntArray& q_in, const KDL::JntArray& dq_in, KDL::Twist& v_out) {
    myik_solver_ptr->JntVelToCartVel(q_in, dq_in, v_out);
  }

  std::string getRobotNs() const {
    return robot_ns;
  }

  int getIndex() {
    return index;
  }
  Affine3d getTransform_base(std::string target);

  Affine3d getT_init() const {
    return T_init;
  }

  Affine3d getT_cur() {
    KDL::JntArray q_cur, dq_cur;
    getState(q_cur, dq_cur);

    KDL::Frame p;
    JntToCart(q_cur, p);

    Affine3d T;
    tf::transformKDLToEigen(p, T);

    return T;
  }

  Affine3d getT_root() {
    return myik_solver_ptr->getT_base_world() * getT_cur();
  }

  Affine3d getT_base_world() {
    return myik_solver_ptr->getT_base_world();
  }

  geometry_msgs::WrenchStamped getForceEef() {
    std::lock_guard<std::mutex> lock(mtx);
    return this->_force;
  }

  unsigned int getNJnt() {
    return nJnt;
  }

  std::vector<std::string> getNameJnt() {
    return nameJnt;
  }

  bool isInitialized() {
    return initialized;
  }

  void setDesired(const KDL::Frame& des_eff_pose, const KDL::Twist& des_eff_vel) {
    std::lock_guard<std::mutex> lock(mtx);
    this->_des_eff_pose = des_eff_pose;
    this->_des_eff_vel = des_eff_vel;
  }

  double freq = 500.0;

  const double eps = 1e-5;

  ros::Publisher markerPublisher, desStatePublisher, curStatePublisher, jntCmdPublisher;
  std::mutex mtx;
  bool flagJntState = false;

  void resetPose();
};

#endif  // CART_CONTROLLER_HPP