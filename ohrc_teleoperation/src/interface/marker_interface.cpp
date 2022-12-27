#include "ohrc_teleoperation/marker_interface.hpp"

MarkerInterface::MarkerInterface() : server("eff_marker") {
}

void MarkerInterface::starting() {
  MultiCartController::starting();
  _markerPose.resize(nRobot);
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
      _flagSubInteractiveMarker[i] = true;
      return;
    }
  }
}

void MarkerInterface::updateManualTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) {
  geometry_msgs::Pose markerPose;
  {
    std::lock_guard<std::mutex> lock(mtx_marker);
    if (!_flagSubInteractiveMarker[controller->getIndex()])
      return;
    markerPose = _markerPose[controller->getIndex()];
  }

  tf2::fromMsg(markerPose, pose);

  //   if (prevPoses[controller->getIndex()].p.Norm() = 0.0)  // initilize
  //     prevPoses[controller->getIndex()] = pose;

  controller->getVelocity(pose, prevPoses[controller->getIndex()], dt, twist);  // TODO: get this velocity in periodic loop using Kalman filter

  prevPoses[controller->getIndex()] = pose;

  controller->enableOperation();
  // update pos and twist
}

#if 0
void MarkerInterface::updateAutoTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) {
  // controller->JntToCart
  // KDL::Frame pos_d;
  // pos_d.p = pos.p + KDL::Vector(-0.15, -0.5, 0.0);
  // static ros::Time t0 = ros::Time::now();
  // double T = 10.0;
  // double s = (ros::Time::now() - t0).toSec() / T, s2, s3, s4, s5;
  // if (s > 1.0)
  //   s = 1.0;
  // s2 = s * s;
  // s3 = s2 * s;
  // s4 = s3 * s;
  // s5 = s4 * s;

  // // min jerk trajectory
  // pos.p = pos.p + (pos_d.p - pos.p) * (6.0 * s5 - 15.0 * s4 + 10.0 * s3);
  // twist.vel = (pos_d.p - pos.p) * (30.0 * s4 - 60.0 * s3 + 30.0 * s2) / T;

  if (MFmode == MFMode::Individual) {
    static ros::Time t0 = ros::Time::now();
    tf::transformEigenToKDL(controller->getT_init(), pose);
    int axis = 0;
    double A = 0.13;
    double f = 0.1;
    double t = (ros::Time::now() - t0).toSec();
    pose.p.data[axis] = -A * cos(2. * M_PI * f * t) + A + pose.p.data[axis];
    twist.vel.data[axis] = A * 2. * M_PI * f * sin(2. * M_PI * f * t);

    axis = 1;
    pose.p.data[axis] = -A * sin(2. * M_PI * f * t) + pose.p.data[axis];
    twist.vel.data[axis] = -A * 2. * M_PI * f * cos(2. * M_PI * f * t);

    A = 0.02;
    f = 0.4;
    axis = 2;
    pose.p.data[axis] = A * cos(2. * M_PI * f * t) - A + pose.p.data[axis];
    twist.vel.data[axis] = -A * 2. * M_PI * f * sin(2. * M_PI * f * t);

    controller->enableOperation();
  } else if (MFmode == MFMode::Parallel) {
    // geometry_msgs::Pose markerPose;
    // {
    //   std::lock_guard<std::mutex> lock(mtx_marker);
    //   if (!_flagSubInteractiveMarker[0])
    //     return;
    //   markerPose = _markerPose[0];
    // }

    // Affine3d T;
    // tf2::fromMsg(markerPose, T);
    // // T = multimyik_solver_ptr->myIKs[controller->getIndex()]->getT_base_world().inverse() * multimyik_solver_ptr->myIKs[0]->getT_base_world() * T;

    // tf::transformEigenToKDL(T, pose);
  }
}
#endif
