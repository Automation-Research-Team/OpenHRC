#ifndef MARKER_INTERFACE_HPP
#define MARKER_INTERFACE_HPP

#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "ohrc_control/interface.hpp"

class MarkerInterface : virtual public Interface {
  visualization_msgs::msg::InteractiveMarker int_marker;

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server;

  void configMarker();
  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback, rclcpp::Logger logger);

  KDL::Frame prevPoses;
  geometry_msgs::msg::Pose _markerPose;
  double _markerDt;
  rclcpp::Time t_prev;

  bool subFirst = false;

  std::mutex mtx_marker;

  bool _flagSubInteractiveMarker = false;
  int count_disable = 0;

public:
  using Interface::Interface;
  virtual void updateTargetPose(const rclcpp::Time t, KDL::Frame& pose, KDL::Twist& twist) override;

  virtual void initInterface() override;
  virtual void resetInterface() override;
};

#endif  // MARKER_INTERFACE_HPP