#ifndef MULTI_MARKER_CONTROLLER_HPP
#define MULTI_MARKER_CONTROLLER_HPP

#include <interactive_markers/interactive_marker_server.h>

#include "ohrc_control/multi_cart_controller.hpp"

class MultiMarkerController : public virtual MultiCartController {
protected:
  interactive_markers::InteractiveMarkerServer server;

  void configMarker(const CartController* cartController);
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  void updateManualTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) override;
  // void updateAutoTargetPose(KDL::Frame& pose, KDL::Twist& twist, CartController* controller) override;

  std::vector<geometry_msgs::Pose> _markerPose;

  std::mutex mtx_marker;

  void starting() override;
  std::vector<bool> _flagSubInteractiveMarker;

public:
  MultiMarkerController();
};

#endif  // MULTI_MARKER_CONTROLLER_HPP