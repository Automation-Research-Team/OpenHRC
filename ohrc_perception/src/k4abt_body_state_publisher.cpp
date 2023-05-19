#include <k4abttypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>

// #include "ohrc_control/arm_marker.hpp"
#include "ohrc_common/coordination_utility.h"
#include "ohrc_common/math_utility.h"
#include "ohrc_common/transform_utility.h"
#include "ohrc_msgs/BodyState.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/MarkerArray.h"

using namespace Eigen;

class ArmPerception {
  enum Body { Shoulder, Elbow, Wrist, NUM };

  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pubMarker, bodyStatePublisher;

  ohrc_msgs::BodyState bodyState;

  tf2_ros::TransformBroadcaster br;

  void cbBodyMarker(const visualization_msgs::MarkerArray::ConstPtr& msg);

  TransformUtility trans;

  Affine3d T_xr_camera;

public:
  ArmPerception();
  int run();
};

ArmPerception::ArmPerception() {
  sub = n.subscribe("/body_tracking_data", 1000, &ArmPerception::cbBodyMarker, this);
  pubMarker = n.advertise<visualization_msgs::MarkerArray>("arm_marker", 1);
  bodyStatePublisher = n.advertise<ohrc_msgs::BodyState>("body_state", 1);

  T_xr_camera = trans.getTransform("xr_frame", "depth_camera_link", ros::Time::now(), ros::Duration(20.0));

  // trans.getTransform("world", )
}

void ArmPerception::cbBodyMarker(const visualization_msgs::MarkerArray::ConstPtr& msg) {
  int nPoint = msg->markers.size();

  for (int i = 0; i < nPoint; i++) {
    int id = msg->markers[i].id % 100;
    if (id == K4ABT_JOINT_WRIST_RIGHT)
      bodyState.right_hand.pose = msg->markers[i].pose;
    else if (id == K4ABT_JOINT_WRIST_LEFT)
      bodyState.left_hand.pose = msg->markers[i].pose;
    else if (id == K4ABT_JOINT_HEAD)
      bodyState.head.pose = msg->markers[i].pose;
  }

  geometry_msgs::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  bodyState.left_hand.twist = twist;
  bodyState.right_hand.twist = twist;
  bodyState.head.twist = twist;

  Affine3d T;
  tf2::fromMsg(bodyState.right_hand.pose, T);
  bodyState.right_hand.pose = tf2::toMsg(T_xr_camera * T);

  tf2::fromMsg(bodyState.left_hand.pose, T);
  bodyState.left_hand.pose = tf2::toMsg(T_xr_camera * T);

  tf2::fromMsg(bodyState.head.pose, T);
  bodyState.head.pose = tf2::toMsg(T_xr_camera * T);

  geometry_msgs::Quaternion quat;
  quat.x = 0.0;
  quat.y = 0.0;
  quat.z = 0.0;
  quat.w = 1.0;

  bodyState.right_hand.pose.orientation = quat;
  bodyState.left_hand.pose.orientation = quat;
  bodyState.head.pose.orientation = quat;

  bodyStatePublisher.publish(bodyState);
}

int ArmPerception::run() {
  ros::spin();
  return 1;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "listener");
  ArmPerception ArmPerception;
  ArmPerception.run();

  return 0;
}
