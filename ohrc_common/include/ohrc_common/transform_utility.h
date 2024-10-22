#ifndef TRANSFORMER_H
#define TRANSFORMER_H

// ros
#include <geometry_msgs/msg/pose_array.hpp>
// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

// tf2
#include <tf2_ros/transform_listener.h>

#include <kdl/frames.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

// eigen3
#include <Eigen/Dense>
using namespace Eigen;

class TransformUtility {
protected:
  // tf2_ros::Buffer tfBuffer;
  // tf2_ros::TransformListener tfListener;
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::TransformListener> tfListener{ nullptr };
  std::unique_ptr<tf2_ros::Buffer> tfBuffer;
  Affine3d Thandle_fp, Thandle_ft, Thandle_lidar, T_l_fp, T_h_fp, T_c_fp, T_color_depth;

public:
  TransformUtility(rclcpp::Node::SharedPtr node) : node(node) {
    tfBuffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
  };

  // (inline) transform velocity vector
  inline static MatrixXd transformVel(VectorXd vel, const Affine3d trans) {
    Vector3d v = trans.rotation() * vel.head(3);
    Vector3d rot = trans.rotation() * vel.tail(3);

    // Vector3d v_ = v + rot.cross(trans.translation());
    Vector3d v_ = (v + rot.cross(trans.translation()));  // TODO check
    Vector3d rot_ = rot;                                 // TODO check

    VectorXd vel_(6);
    vel_ << v_, rot_;

    return vel_;
  }

  // (inline) transform force & torque vector
  inline static MatrixXd transformFT(VectorXd ft_in, const Affine3d trans) {
    Vector3d force_out, torque_out;
    TransformUtility::transformFT(ft_in.head(3), ft_in.tail(3), trans, force_out, torque_out);

    VectorXd ft_out(6);
    ft_out.head(3) = force_out;
    ft_out.tail(3) = torque_out;

    return ft_out;
  }

  // (inline) transform force & torque vector
  inline static void transformFT(Vector3d force_in, Vector3d torque_in, const Affine3d trans, Vector3d& force_out, Vector3d& torque_out) {
    force_out = trans.rotation() * force_in;
    torque_out = trans.rotation() * torque_in + trans.translation().cross(force_out);  // TODO check
  }

  // get transform as Eigen::affine3d
  inline Affine3d getTransform(const std::string& target, const std::string& source, const rclcpp::Time& time, rclcpp::Duration timeout) {
    return tf2::transformToEigen(tfBuffer->lookupTransform(target, source, time, timeout));
  }

  // check if transform is possible
  inline bool canTransform(const std::string& target, const std::string& source, const rclcpp::Time& time, rclcpp::Duration timeout) {
    return tfBuffer->canTransform(target, source, time, timeout);
  }
};

namespace tf2 {

inline void fromMsg(const geometry_msgs::msg::Twist& msg, KDL::Twist& out) {
  out.vel[0] = msg.linear.x;
  out.vel[1] = msg.linear.y;
  out.vel[2] = msg.linear.z;
  out.rot[0] = msg.angular.x;
  out.rot[1] = msg.angular.y;
  out.rot[2] = msg.angular.z;
}

inline void fromMsg(const geometry_msgs::msg::PoseArray& msg, std::vector<Affine3d>& Ts) {
  Ts.resize(msg.poses.size());
  for (int i = 0; i < msg.poses.size(); i++)
    tf2::fromMsg(msg.poses[i], Ts[i]);
}
}  // namespace tf2

#endif  // TRANSFORMER_H
