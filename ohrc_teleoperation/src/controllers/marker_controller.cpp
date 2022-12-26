#include "ohrc_control/marker_controller.hpp"

MarkerController::MarkerController() : server(robot_ns + "eff_marker") {
}

void MarkerController::initWithJnt(const KDL::JntArray& q_init) {
  configMarker(q_init);
}

void MarkerController::configMarker(const KDL::JntArray& q_init) {
  KDL::Frame init_eff_pose;
  // fk_solver_ptr->JntToCart(q_init, init_eff_pose);
  tf::transformEigenToKDL(T_init, init_eff_pose);

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = robot_ns + chain_start;
  int_marker.header.stamp = ros::Time(0);
  int_marker.pose = tf2::toMsg(init_eff_pose);
  int_marker.scale = 0.1;
  // int_marker.name = "simple_6dof";
  // int_marker.description = "Simple 6-DOF Control";

  // insert a box
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = int_marker.scale * 0.45;
  box_marker.scale.y = int_marker.scale * 0.45;
  box_marker.scale.z = int_marker.scale * 0.45;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back(box_marker);

  int_marker.controls.push_back(box_control);
  int_marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  visualization_msgs::InteractiveMarkerControl control;

  const std::string axis_name[3] = { "x", "y", "z" };
  const Eigen::Quaterniond quat[3] = { Eigen::Quaterniond(1, 1, 0, 0), Eigen::Quaterniond(1, 0, 0, 1), Eigen::Quaterniond(1, 0, 1, 0) };

  for (int i = 0; i < 3; i++) {
    control.orientation = tf2::toMsg(quat[i].normalized());
    control.name = "rotate_" + axis_name[i];
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_" + axis_name[i];
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker, boost::bind(&MarkerController::processFeedback, this, _1));

  // 'commit' changes and send to all clients
  server.applyChanges();
}

void MarkerController::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  // ROS_INFO_STREAM(feedback->pose);

  std::lock_guard<std::mutex> lock(mtx);
  tf2::fromMsg(feedback->pose, this->_des_eff_pose);

  static ros::Time prev_time = ros::Time::now();
  static KDL::Frame prev_des_eff_pose = _des_eff_pose;

  ros::Time current_time = ros::Time::now();
  double dt = (current_time - prev_time).toSec();

  if (dt < 0.01)
    return;

  getVelocity(_des_eff_pose, prev_des_eff_pose, dt, _des_eff_vel);  // TODO: get this velocity in periodic loop using Kalman filter

  if (!flagEffPose)
    flagEffPose = true;

  prev_time = current_time;
  prev_des_eff_pose = _des_eff_pose;

  if (dt > 1.0)
    _disable = true;
  else
    _disable = false;

  // Matrix<double, 6, 1> vel(6);
  // Vector3d p;

  // tf::twistKDLToEigen(_des_eff_vel, vel);
  // tf::vectorKDLToEigen(_des_eff_pose.p, p);
  // std::cout << p.transpose() << std::endl;
}