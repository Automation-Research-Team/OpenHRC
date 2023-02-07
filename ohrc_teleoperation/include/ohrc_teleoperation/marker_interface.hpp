#ifndef MARKER_INTERFACE_HPP
#define MARKER_INTERFACE_HPP

#include <interactive_markers/interactive_marker_server.h>

#include "ohrc_control/interface.hpp"

class MarkerInterface : public Interface {
  visualization_msgs::InteractiveMarker int_marker;

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server;

  void configMarker();
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  KDL::Frame prevPoses;
  geometry_msgs::Pose _markerPose;
  double _markerDt;
  ros::Time t_prev;

  std::mutex mtx_marker;

  bool _flagSubInteractiveMarker = false;

public:
  using Interface::Interface;
  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override;

  void initInterface() override;
  void resetInterface() override;
};

#endif  // MARKER_INTERFACE_HPP