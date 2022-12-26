#ifndef MARKER_CONTROLLER_HPP
#define MARKER_CONTROLLER_HPP

#include <interactive_markers/interactive_marker_server.h>

#include "ohrc_control/cart_controller.hpp"

class MarkerController : public virtual CartController {
protected:
  interactive_markers::InteractiveMarkerServer server;

  virtual void initWithJnt(const KDL::JntArray& q_init) override;

  void configMarker(const KDL::JntArray& q_init);
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

public:
  MarkerController();
};

#endif  // MARKER_CONTROLLER_HPP