#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "ohrc_msgs/BodyState.h"

class BodyStatePrediction {
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher bodyStateMarkerPublisher;

  ohrc_msgs::BodyState _bodyState;

  std::mutex mtx;

  void cbBodyState(const ohrc_msgs::BodyState::ConstPtr& msg);
  void publishBodyStateMarker(const ohrc_msgs::BodyState& bodyState);
  void predictBodyState(ohrc_msgs::BodyState& bodyState);

public:
  BodyStatePrediction();
  int run();
};

BodyStatePrediction::BodyStatePrediction() {
  sub = n.subscribe("/body_state", 1000, &BodyStatePrediction::cbBodyState, this);
  bodyStateMarkerPublisher = n.advertise<visualization_msgs::MarkerArray>("/body_state/marker", 1);
}

void BodyStatePrediction::cbBodyState(const ohrc_msgs::BodyState::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx);
  _bodyState = *msg;
}

void BodyStatePrediction::publishBodyStateMarker(const ohrc_msgs::BodyState& bodyState) {
  visualization_msgs::MarkerArray markerArray;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "xr_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "right_hand";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = bodyState.right_hand.pose;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  markerArray.markers.push_back(marker);

  marker.ns = "left_hand";
  marker.id = 1;
  marker.pose = bodyState.left_hand.pose;
  marker.color.g = 1.0;
  markerArray.markers.push_back(marker);

  marker.ns = "head";
  marker.id = 2;
  marker.pose = bodyState.head.pose;
  marker.color.r = 0.0;
  marker.color.b = 1.0;
  markerArray.markers.push_back(marker);

  bodyStateMarkerPublisher.publish(markerArray);
}

void BodyStatePrediction::predictBodyState(ohrc_msgs::BodyState& bodyState) {
  // predict body state
}

int BodyStatePrediction::run() {
  ros::Rate r(50);
  ohrc_msgs::BodyState bodyState;
  while (ros::ok()) {
    {
      std::lock_guard<std::mutex> lock(mtx);
      bodyState = _bodyState;
    }
    predictBodyState(bodyState);
    publishBodyStateMarker(bodyState);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "body_state_visualizer");
  BodyStatePrediction BodyStatePrediction;
  BodyStatePrediction.run();

  return 0;
}