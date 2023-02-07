#ifndef MARKER_INTERFACE_HPP
#define MARKER_INTERFACE_HPP

#include <interactive_markers/interactive_marker_server.h>

#include "ohrc_control/cart_controller.hpp"

class MarkerInterface {
  std::shared_ptr<CartController> controller;
  visualization_msgs::InteractiveMarker int_marker;

protected:
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server;

  void configMarker();
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  // void updateAutoTargetPose(KDL::Frame& pose, KDL::Twist& twist, std::shared_ptr<CartController> controller) override;

  KDL::Frame prevPoses;
  geometry_msgs::Pose _markerPose;
  double _markerDt;
  ros::Time t_prev;

  std::mutex mtx_marker;

  void starting();
  bool _flagSubInteractiveMarker = false;

public:
  MarkerInterface(std::shared_ptr<CartController> controller);
  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist);
  void resetInterface();
};

#endif  // MARKER_INTERFACE_HPP