#include <k4abttypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>

#include "ohrc_control/arm_marker.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "toroboarm_common/coordination_utility.h"
#include "toroboarm_common/math_utility.h"
#include "toroboarm_common/transform_utility.h"
#include "visualization_msgs/MarkerArray.h"

using namespace Eigen;

class ArmPerception {
  enum Body { Shoulder, Elbow, Wrist, NUM };

  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pubMarker;

  tf2_ros::TransformBroadcaster br;

  void cbBodyMarker(const visualization_msgs::MarkerArray::ConstPtr& msg);

  TransformUtility trans;

public:
  ArmPerception();
  int run();
};

ArmPerception::ArmPerception() {
  sub = n.subscribe("/body_tracking_data", 1000, &ArmPerception::cbBodyMarker, this);
  pubMarker = n.advertise<visualization_msgs::MarkerArray>("arm_marker", 1);

  // trans.getTransform("world", )
}

void ArmPerception::cbBodyMarker(const visualization_msgs::MarkerArray::ConstPtr& msg) {
  int nPoint = msg->markers.size();
  std::vector<int> ids(Body::NUM, -1);
  for (int i = 0; i < nPoint; i++) {
    int id = msg->markers[i].id % 100;
    if (id == K4ABT_JOINT_SHOULDER_RIGHT)
      ids[Body::Shoulder] = i;
    else if (id == K4ABT_JOINT_ELBOW_RIGHT)
      ids[Body::Elbow] = i;
    else if (id == K4ABT_JOINT_WRIST_RIGHT)
      ids[Body::Wrist] = i;
  }
  // ids[Body::Shoulder] = K4ABT_JOINT_SHOULDER_RIGHT;  // TODO: Always? -> probably yes, but need to check detection succsess in another way.
  // ids[Body::Elbow] = K4ABT_JOINT_ELBOW_RIGHT;
  // ids[Body::Wrist] = K4ABT_JOINT_WRIST_RIGHT;

  if (ids[Body::Shoulder] == -1 || ids[Body::Elbow] == -1 || ids[Body::Wrist] == -1)
    return;
  // std::cout << "shoulder: " << ids[Body::Shoulder] << ", elbow: " << ids[Body::Elbow] << ", wrist: " << ids[Body::Wrist] << std::endl;

  static Affine3d T_depth_camera = trans.getTransform(msg->markers[ids[Body::Shoulder]].header.frame_id, "camera_base", ros::Time::now(), ros::Duration(1.0));
  static Matrix3d R_shoulder_depth = T_depth_camera.rotation() * AngleAxisd(M_PI, Vector3d::UnitZ());

  std::vector<Eigen::Affine3d> T(Body::NUM);
  for (int i = 0; i < Body::NUM; i++)
    tf2::fromMsg(msg->markers[ids[i]].pose, T[i]);

  // std::cout << "shoulder: " << T[Body::Shoulder].translation().transpose() << ", elbow: " << T[Body::Elbow].translation().transpose() << ", wrist: " << T[Body::Wrist].translation().transpose() << std::endl;

  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = msg->markers[ids[Body::Shoulder]].header.stamp;
  transformStamped.header.frame_id = msg->markers[ids[Body::Shoulder]].header.frame_id;
  transformStamped.child_frame_id = "arm_origin";
  transformStamped.transform.translation.x = msg->markers[ids[Body::Shoulder]].pose.position.x;
  transformStamped.transform.translation.y = msg->markers[ids[Body::Shoulder]].pose.position.y;
  transformStamped.transform.translation.z = msg->markers[ids[Body::Shoulder]].pose.position.z;
  transformStamped.transform.rotation.x = Quaterniond(R_shoulder_depth).x();
  transformStamped.transform.rotation.y = Quaterniond(R_shoulder_depth).y();
  transformStamped.transform.rotation.z = Quaterniond(R_shoulder_depth).z();
  transformStamped.transform.rotation.w = Quaterniond(R_shoulder_depth).w();
  br.sendTransform(transformStamped);

  std::vector<Eigen::Vector3d> p(2);
  std::vector<double> l(2);
  for (int i = 0; i < Body::NUM - 1; i++) {
    p[i] = R_shoulder_depth.transpose() * (T[i + 1].translation() - T[i].translation());
    l[i] = p[i].norm();
  }

  // std::cout << p[0].transpose() << std::endl;

  std::vector<double> theta(2), phi(2);
  theta[0] = atan2(p[0].y(), p[0].x());
  theta[1] = atan2(p[1].y(), p[1].x()) - theta[0];

  phi[0] = atan2(p[0].z(), p[0].head(2).norm());
  phi[1] = atan2(p[1].z(), p[1].head(2).norm()) - phi[0];

  // std::cout << "theta[0]: " << theta[0] << ", theta[1]: " << theta[1] << std::endl;
  // std::cout << "  phi[0]: " << phi[0] << ",   phi[1]: " << phi[1] << std::endl;

  Vector3d x;
  x(0) = l[0] * cos(phi[0]) * cos(theta[0]) + l[1] * cos(phi[0] + phi[1]) * cos(theta[0] + theta[1]);
  x(1) = l[0] * cos(phi[0]) * sin(theta[0]) + l[1] * cos(phi[0] + phi[1]) * sin(theta[0] + theta[1]);
  x(2) = l[0] * sin(phi[0]) + l[1] * sin(phi[0] + phi[1]);

  MatrixXd J(3, 4);
  // dx/ dtheta[0]
  J(0, 0) = -l[0] * cos(phi[0]) * sin(theta[0]) - l[1] * cos(phi[0] + phi[1]) * sin(theta[0] + theta[1]);
  J(1, 0) = l[0] * cos(phi[0]) * cos(theta[0]) + l[1] * cos(phi[0] + phi[1]) * cos(theta[0] + theta[1]);
  J(2, 0) = 0.0;

  // dx/ dphi[0]
  J(0, 1) = -l[0] * sin(phi[0]) * cos(theta[0]) - l[1] * sin(phi[0] + phi[1]) * cos(theta[0] + theta[1]);
  J(1, 1) = -l[0] * sin(phi[0]) * sin(theta[0]) - l[1] * sin(phi[0] + phi[1]) * sin(theta[0] + theta[1]);
  J(2, 1) = l[0] * cos(phi[0]) + l[1] * cos(phi[0] + phi[1]);

  // dx/ dtheta[1]
  J(0, 2) = -l[1] * cos(phi[0] + phi[1]) * sin(theta[0] + theta[1]);
  J(1, 2) = l[1] * cos(phi[0] + phi[1]) * cos(theta[0] + theta[1]);
  J(2, 2) = 0.0;

  // dx/ dphi[1]
  J(0, 3) = -l[1] * sin(phi[0] + phi[1]) * cos(theta[0] + theta[1]);
  J(1, 3) = -l[1] * sin(phi[0] + phi[1]) * sin(theta[0] + theta[1]);
  J(2, 3) = l[1] * cos(phi[0] + phi[1]);

  Eigen::JacobiSVD<MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
  VectorXd s = svd.singularValues();
  MatrixXd U = svd.matrixU();

  // std::cout << U.determinant() << std::endl;

  if (U.determinant() < 0)
    U.col(2) = -U.col(2);

  // std::cout << U.determinant() << std::endl;
  // std::cout << "---" << std::endl;

  Quaterniond q = Quaterniond(Matrix3d(U)).normalized();

  visualization_msgs::Marker armMarker;
  armMarker.header.frame_id = "arm_origin";
  armMarker.header.stamp = msg->markers[ids[Body::Shoulder]].header.stamp;
  armMarker.id = ArmMarker::ArmMarkerID::Skeleton;
  armMarker.type = visualization_msgs::Marker::LINE_STRIP;
  armMarker.action = visualization_msgs::Marker::ADD;
  armMarker.points.resize(3);
  armMarker.points[1] = tf2::toMsg(p[0]);
  armMarker.points[2] = tf2::toMsg(Vector3d(p[0] + p[1]));
  armMarker.pose.orientation.w = 1.0;
  armMarker.color.a = 1.0;  // Don't forget to set the alpha!
  armMarker.color.r = 0.0;
  armMarker.color.g = 1.0;
  armMarker.color.b = 0.0;
  armMarker.scale.x = 0.01;
  armMarker.scale.y = 0.01;
  armMarker.scale.z = 0.01;

  visualization_msgs::Marker manipuMarker;
  manipuMarker.header.frame_id = "arm_origin";
  manipuMarker.header.stamp = msg->markers[ids[Body::Shoulder]].header.stamp;
  manipuMarker.id = ArmMarker::ArmMarkerID::ManipulabilityEllipsoid;
  manipuMarker.type = visualization_msgs::Marker::SPHERE;
  manipuMarker.action = visualization_msgs::Marker::ADD;
  manipuMarker.pose.position = tf2::toMsg(Vector3d(p[0] + p[1]));
  manipuMarker.pose.orientation = tf2::toMsg(q);
  manipuMarker.color.a = 0.5;  // Don't forget to set the alpha!
  manipuMarker.color.r = 1.0;
  manipuMarker.color.g = 0.0;
  manipuMarker.color.b = 0.0;
  tf2::toMsg(Vector3d(s), manipuMarker.scale);

  visualization_msgs::MarkerArray markers;
  markers.markers.push_back(armMarker);
  markers.markers.push_back(manipuMarker);

  pubMarker.publish(markers);
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
