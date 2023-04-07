#include "ohrc_teleoperation/marker_interface.hpp"

void MarkerInterface::initInterface() {
  server.reset(new interactive_markers::InteractiveMarkerServer(controller->getRobotNs() + "eff_marker"));
  configMarker();
}

void MarkerInterface::configMarker() {
  int_marker.header.frame_id = controller->getRobotNs() + controller->getChainStart();
  int_marker.header.stamp = ros::Time(0);
  int_marker.pose = tf2::toMsg(controller->getT_init());

  // int_marker.pose.position = tf2::toMsg(Vector3d(cartController->getT_init().translation()));
  // int_marker.pose.orientation = tf2::toMsg(Quaterniond(cartController->getT_init().rotation().transpose()));
  int_marker.scale = 0.1;
  int_marker.name = controller->getRobotNs();

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
  server->insert(int_marker);
  server->setCallback(int_marker.name, boost::bind(&MarkerInterface::processFeedback, this, _1));

  // 'commit' changes and send to all clients
  server->applyChanges();

  ROS_INFO_STREAM("Set interactive marker: " << controller->getRobotNs() << "eff_marker");
}

void MarkerInterface::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  std::lock_guard<std::mutex> lock(mtx_marker);
  _markerPose = feedback->pose;
  _markerDt = (ros::Time::now() - t_prev).toSec();
  _flagSubInteractiveMarker = true;
  t_prev = ros::Time::now();
}

void MarkerInterface::updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) {
  geometry_msgs::Pose markerPose;
  // controller->enableOperation();
  double markerDt;
  {
    std::lock_guard<std::mutex> lock(mtx_marker);
    if (!_flagSubInteractiveMarker) {
      // controller->disableOperation();
      return;
    }
    // _flagSubInteractiveMarker[controller->getIndex()] = false;
    markerPose = _markerPose;
    // markerDt = _markerDt[controller->getIndex()];
    controller->enableOperation();
  }

  tf2::fromMsg(markerPose, pose);

  if (prevPoses.p.data[0] == 0.0 && prevPoses.p.data[1] == 0.0 && prevPoses.p.data[2] == 0.0)  // initilize
    prevPoses = pose;

  // controller->getVelocity(pose, prevPoses, dt, twist);  // TODO: get this velocity in periodic loop using Kalman filter

  prevPoses = pose;
}

void MarkerInterface::resetInterface() {
  ROS_WARN_STREAM("Reset marker position");
  server->setPose(int_marker.name, int_marker.pose);
  server->applyChanges();

  _markerPose = int_marker.pose;
  _flagSubInteractiveMarker = false;
}