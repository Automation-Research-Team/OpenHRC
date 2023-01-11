#include "ohrc_teleoperation/marker_interface.hpp"

MarkerInterface::MarkerInterface() : server("eff_marker") {
}

void MarkerInterface::starting() {
  MultiCartController::starting();
  _markerPose.resize(nRobot);
  _markerDt.resize(nRobot);
  t_prev.resize(nRobot, ros::Time::now());
  prevPoses.resize(nRobot);
  _flagSubInteractiveMarker.resize(nRobot, false);
  for (auto& ind : manualInd)
    configMarker(cartControllers[ind].get());
  server.applyChanges();
}

void MarkerInterface::configMarker(const CartController* cartController) {
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = cartController->getRobotNs() + cartController->getChainStart();
  int_marker.header.stamp = ros::Time(0);
  int_marker.pose = tf2::toMsg(cartController->getT_init());

  // int_marker.pose.position = tf2::toMsg(Vector3d(cartController->getT_init().translation()));
  // int_marker.pose.orientation = tf2::toMsg(Quaterniond(cartController->getT_init().rotation().transpose()));
  int_marker.scale = 0.1;
  int_marker.name = cartController->getRobotNs();

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
  //   server.insert(int_marker, boost::bind(&MarkerInterface::processFeedback, this, _1));
  server.insert(int_marker);
  server.setCallback(int_marker.name, boost::bind(&MarkerInterface::processFeedback, this, _1));

  // 'commit' changes and send to all clients
  // server.applyChanges();
}

void MarkerInterface::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  std::lock_guard<std::mutex> lock(mtx_marker);
  // ROS_INFO_STREAM(*feedback);

  for (int i = 0; i < nRobot; i++) {
    if (feedback->marker_name == cartControllers[i]->getRobotNs()) {
      _markerPose[i] = feedback->pose;
      _markerDt[i] = (ros::Time::now() - t_prev[i]).toSec();
      _flagSubInteractiveMarker[i] = true;
      t_prev[i] = ros::Time::now();
      return;
    }
  }
}

void MarkerInterface::updateManualTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) {
  geometry_msgs::Pose markerPose;
  double markerDt;
  {
    std::lock_guard<std::mutex> lock(mtx_marker);
    if (!_flagSubInteractiveMarker[controller->getIndex()])
      return;
    // _flagSubInteractiveMarker[controller->getIndex()] = false;
    markerPose = _markerPose[controller->getIndex()];
    // markerDt = _markerDt[controller->getIndex()];
  }

  tf2::fromMsg(markerPose, pose);

  if (prevPoses[controller->getIndex()].p.data[0] == 0.0 && prevPoses[controller->getIndex()].p.data[1] == 0.0 && prevPoses[controller->getIndex()].p.data[2] == 0.0)  // initilize
    prevPoses[controller->getIndex()] = pose;

  controller->getVelocity(pose, prevPoses[controller->getIndex()], dt, twist);  // TODO: get this velocity in periodic loop using Kalman filter

  prevPoses[controller->getIndex()] = pose;

  controller->enableOperation();
  // update pos and twist
}