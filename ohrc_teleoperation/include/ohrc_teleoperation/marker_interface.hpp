#ifndef MARKER_INTERFACE_HPP
#define MARKER_INTERFACE_HPP

#include <interactive_markers/interactive_marker_server.h>

#include "ohrc_control/multi_cart_controller.hpp"

class MarkerInterface : public virtual MultiCartController {
  visualization_msgs::InteractiveMarker int_marker;

protected:
  interactive_markers::InteractiveMarkerServer server;

  void configMarker(const CartController* cartController);
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) override;
  // void updateAutoTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) override;

  std::vector<KDL::Frame> prevPoses;
  std::vector<geometry_msgs::Pose> _markerPose;
  std::vector<double> _markerDt;
  std::vector<ros::Time> t_prev;

  std::mutex mtx_marker;

  void starting() override;
  std::vector<bool> _flagSubInteractiveMarker;

  void resetInterface() override;

public:
  MarkerInterface();
};

#endif  // MARKER_INTERFACE_HPP