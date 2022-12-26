#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Geometry>

std::vector<ros::Publisher> pub;
int nRobot = 9;

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  for (int i = 0; i < nRobot; i++)
    pub[i].publish(feedback->pose);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_marker");
  ros::NodeHandle nh;

  pub.resize(nRobot);
  for (int i = 0; i < nRobot; i++)
    pub[i] = nh.advertise<geometry_msgs::Pose>("/toroboarm_" + std::to_string(i + 1) + "/cmd_pose", 2);

  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("simple_marker");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = "my_marker";
  int_marker.description = "";
  int_marker.scale = 0.1;

  int_marker.pose.position.z = 1.5;

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
  server.insert(int_marker, &processFeedback);

  // 'commit' changes and send to all clients
  server.applyChanges();

  // start the ROS main loop
  ros::spin();
}