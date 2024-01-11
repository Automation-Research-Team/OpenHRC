#include "ohrc_automation/joint_trajectory_controller.hpp"

void JointTrajectoryController::initInterface() {
  this->setSubscriber();
}

void JointTrajectoryController::setSubscriber() {
  trjSubscriber = n.subscribe<moveit_msgs::MoveGroupActionResult>("/move_group/result", 1000, &JointTrajectoryController::cbJointTrajectory, this, th);
}

void JointTrajectoryController::cbJointTrajectory(const moveit_msgs::MoveGroupActionResult::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_joint);
  _jointTrj = msg->result.planned_trajectory.joint_trajectory;
  _cartTrj.points.resize(_jointTrj.points.size());

  std::vector<std::string> name = controller->getNameJnt();
  for (int i = 0; i < _jointTrj.points.size(); i++) {
    int j;
    std_utility::exist_in(name, _jointTrj.joint_names[i], j);

    KDL::JntArray q_in(controller->getNJnt()), dq_in(controller->getNJnt());
    q_in.data = VectorXd::Map(&_jointTrj.points[i].positions.data()[0], _jointTrj.points[i].positions.size());
    dq_in.data = VectorXd::Map(&_jointTrj.points[i].velocities.data()[0], _jointTrj.points[i].velocities.size());

    KDL::Frame p_out;
    KDL::Twist v_out;
    controller->JntToCart(q_in, dq_in, p_out, v_out);
    _cartTrj.points[j].point.pose = tf2::toMsg(p_out);
    _cartTrj.points[j].point.velocity.linear.x = v_out.vel.data[0];
    _cartTrj.points[j].point.velocity.linear.y = v_out.vel.data[1];
    _cartTrj.points[j].point.velocity.linear.z = v_out.vel.data[2];
    _cartTrj.points[j].point.velocity.angular.x = v_out.rot.data[0];
    _cartTrj.points[j].point.velocity.angular.y = v_out.rot.data[1];
    _cartTrj.points[j].point.velocity.angular.z = v_out.rot.data[2];

    // _cartTrj.points[j].point.velocity = _jointTrj.points[i].velocities[j];
    _cartTrj.points[j].time_from_start = _jointTrj.points[i].time_from_start;
    // std::cout << p_out.p.data[0] << " " << p_out.p.data[1] << " " << p_out.p.data[2] << std::endl;
  }

  _start = true;
  // std::cout << "subscribed" << std::endl;
}

void JointTrajectoryController::updateTargetPose(KDL::Frame& pos, KDL::Twist& twist) {
  trajectory_msgs::JointTrajectory joint_trj;
  moveit_msgs::CartesianTrajectory cartTrj;

  bool start;
  {
    std::lock_guard<std::mutex> lock(mtx_joint);

    if (_cartTrj.points.empty())
      return;
    cartTrj = _cartTrj;
    joint_trj = _jointTrj;
    start = _start;
    _start = false;
  }

  // static bool start = true;
  static ros::Time t_start;
  static int i = 0;
  if (start) {
    t_start = ros::Time::now();
    i = 0;
    start = false;
  }

  bool stop = false;
  if (i >= cartTrj.points.size()) {
    stop = true;
  }

  tf2::fromMsg(cartTrj.points[i].point.pose, pos);
  tf2::fromMsg(cartTrj.points[i].point.velocity, twist);

  KDL::JntArray q_rest(controller->getNJnt());
  q_rest.data = VectorXd::Map(&joint_trj.points[i].positions.data()[0], joint_trj.points[i].positions.size());
  controller->setqRest(q_rest);

  if (stop) {
    controller->disableOperation();
    return;
  } else
    controller->enableOperation();

  if ((ros::Time::now() - t_start).toNSec() > cartTrj.points[i].time_from_start.toNSec() && i < cartTrj.points.size() - 1)
    i++;
}
