#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "ohrc_msgs/BodyState.h"

class BodyStateVisualizer {
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher bodyStateMarkerPublisher;

  ohrc_msgs::BodyState _bodyState;

  void cbBodyState(const ohrc_msgs::BodyState::ConstPtr& msg);

public:
  BodyStateVisualizer();
  int run();
};

BodyStateVisualizer::BodyStateVisualizer() {
  sub = n.subscribe("/body_state", 1000, &BodyStateVisualizer::cbBodyState, this);
  bodyStateMarkerPublisher = n.advertise<visualization_msgs::MarkerArray>("/body_state/marker", 1);
}

void BodyStateVisualizer::cbBodyState(const ohrc_msgs::BodyState::ConstPtr& msg) {
  _bodyState = *msg;

  visualization_msgs::MarkerArray markerArray;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "xr_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "right_hand";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = _bodyState.right_hand.pose;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  markerArray.markers.push_back(marker);

  marker.ns = "left_hand";
  marker.id = 1;
  marker.pose = _bodyState.left_hand.pose;
  marker.color.g = 1.0;
  markerArray.markers.push_back(marker);

  marker.ns = "head";
  marker.id = 2;
  marker.pose = _bodyState.head.pose;
  marker.color.r = 0.0;
  marker.color.b = 1.0;
  markerArray.markers.push_back(marker);

  bodyStateMarkerPublisher.publish(markerArray);
}

int BodyStateVisualizer::run() {
  ros::spin();

  return 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "body_state_visualizer");
  BodyStateVisualizer BodyStateVisualizer;
  BodyStateVisualizer.run();

  return 0;
}