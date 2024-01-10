#ifndef ODOMETRY_UTILITY_H
#define ODOMETRY_UTILITY_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

// tf2
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Dense>

namespace nav_msgs_utility {
class Odometry {
public:
  inline void toVector_3dof(nav_msgs::Odometry odom, Eigen::MatrixXd &vector) {
    vector(0) = odom.pose.pose.position.x;                //[m]
    vector(1) = odom.pose.pose.position.y;                //[m]
    vector(2) = tf2::getYaw(odom.pose.pose.orientation);  //[rad]
    vector(3) = odom.twist.twist.linear.x;                //[m/s]
    vector(4) = odom.twist.twist.linear.y;                //[m/s]
    vector(5) = odom.twist.twist.angular.z;               //[rad/s]
  }

  inline void toVector_6dof(nav_msgs::Odometry odom, Eigen::MatrixXd &vector) {
    vector(0) = odom.pose.pose.position.x;                                          //[m]
    vector(1) = odom.pose.pose.position.y;                                          //[m]
    vector(2) = odom.pose.pose.position.z;                                          //[m]
    tf2::getEulerYPR(odom.pose.pose.orientation, vector(5), vector(4), vector(3));  //[rad/s] (yaw, pitch, roll)
    vector(6) = odom.twist.twist.linear.x;                                          //[m/s]
    vector(7) = odom.twist.twist.linear.y;                                          //[m/s]
    vector(8) = odom.twist.twist.linear.z;                                          //[m/s]
    vector(9) = odom.twist.twist.angular.x;                                         //[rad/s]
    vector(10) = odom.twist.twist.angular.y;                                        //[rad/s]
    vector(11) = odom.twist.twist.angular.z;                                        //[rad/s]
  }

  inline void fromVector_3dof(Eigen::MatrixXd vector, nav_msgs::Odometry &odom) {
    odom.pose.pose.position.x = vector(0);                                                                                //[m]
    odom.pose.pose.position.y = vector(1);                                                                                //[m]
    odom.pose.pose.orientation = tf2::toMsg(Eigen::Quaterniond(Eigen::AngleAxisd(vector(2), Eigen::Vector3d::UnitZ())));  //[quaternion]
    odom.twist.twist.linear.x = vector(3);                                                                                //[m/s]
    odom.twist.twist.linear.y = vector(4);                                                                                //[m/s]
    odom.twist.twist.angular.z = vector(5);                                                                               //[rad/s]
  }

  inline void fromVector_6dof(Eigen::MatrixXd vector, nav_msgs::Odometry &odom) {
    odom.pose.pose.position.x = vector(0);  //[m]
    odom.pose.pose.position.y = vector(1);  //[m]
    odom.pose.pose.position.z = vector(2);  //[m]
    odom.pose.pose.orientation = tf2::toMsg(Eigen::Quaterniond(Eigen::AngleAxisd(vector(3), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(vector(4), Eigen::Vector3d::UnitY()) *
                                                               Eigen::AngleAxisd(vector(5), Eigen::Vector3d::UnitZ())));  //[quaternion]
    odom.twist.twist.linear.x = vector(6);                                                                                //[m/s]
    odom.twist.twist.linear.y = vector(7);                                                                                //[m/s]
    odom.twist.twist.linear.z = vector(8);                                                                                //[m/s]
    odom.twist.twist.angular.x = vector(9);                                                                               //[rad/s]
    odom.twist.twist.angular.y = vector(10);                                                                              //[rad/s]
    odom.twist.twist.angular.z = vector(11);                                                                              //[rad/s]
  }

  inline void transFromOdom(nav_msgs::Odometry odom, geometry_msgs::TransformStamped &trans_base_odom) {
    Eigen::Affine3d T_base_odom;
    tf2::fromMsg(odom.pose.pose, T_base_odom);
    trans_base_odom = tf2::eigenToTransform(T_base_odom);
  }

  inline void transToOdom(nav_msgs::Odometry odom, geometry_msgs::TransformStamped &trans_odom_base) {
    Eigen::Affine3d T_base_odom;
    tf2::fromMsg(odom.pose.pose, T_base_odom);
    Eigen::Affine3d T_odom_base = T_base_odom.inverse();
    trans_odom_base = tf2::eigenToTransform(T_odom_base);
  }
};
}  // namespace nav_msgs_utility

#endif  // ODOMETRY_UTILITY_H
