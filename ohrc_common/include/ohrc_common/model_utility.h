#ifndef MODEL_UTILITY_H
#define MODEL_UTILITY_H

#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ohrc_common/rclcpp_utility.hpp"

namespace ModelUtility {
// std::shared_ptr<rclcpp::Node> node_;

inline urdf::Model getURDFModel(const rclcpp::Node::SharedPtr &node, const std::string URDF_param) {
  urdf::Model robot_model;
  std::string urdf_string, urdf_xml, full_urdf_xml;
  // nh.param("urdf_xml", urdf_xml, URDF_param);
  // RclcppUtility::declare_and_get_parameter(node, "urdf_xml", URDF_param, urdf_xml);
  // nh.searchParam(urdf_xml, full_urdf_xml)
  full_urdf_xml = URDF_param;

  // if (!RclcppUtility::declare_and_get_parameter(node, full_urdf_xml, xml_string, ) {
  //   ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
  //   return robot_model;
  // }

  // nh.param(full_urdf_xml, xml_string, std::string());
  // RclcppUtility::declare_and_get_parameter(node, full_urdf_xml, std::string(), xml_string, false);
  auto param_client_ = std::make_shared<rclcpp::SyncParametersClient>(node, "/robot_state_publisher");

  using namespace std::chrono_literals;
  while (!param_client_->wait_for_service(1s)) {
    RCLCPP_INFO(node->get_logger(), "waiting for service");
  }

  auto params = param_client_->get_parameters({ URDF_param });

  for (auto &param : params) {
    if (param.get_name() == URDF_param) {
      robot_model.initString(param.value_to_string());
      break;
    }
  }

  return robot_model;
}

inline KDL::Chain getKDLChain(const urdf::Model &robot_model, const std::string &base_link, const std::string &tip_link) {
  // ROS_DEBUG_STREAM("Reading joints and links from URDF");

  KDL::Chain chain;
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(robot_model, tree)) {
  }
  // ROS_FATAL("Failed to extract kdl tree from xml robot description");

  if (!tree.getChain(base_link, tip_link, chain)) {
  }
  // ROS_FATAL("Couldn't find chain %s to %s", base_link.c_str(), tip_link.c_str());

  return chain;
}

inline void getBounds(const urdf::Model &robot_model, const KDL::Chain &chain, const double mergin, KDL::JntArray &lb, KDL::JntArray &ub, KDL::JntArray &vb) {
  std::vector<KDL::Segment> chain_segs = chain.segments;
  uint joint_num = 0;
  for (unsigned int i = 0; i < chain_segs.size(); ++i) {
    urdf::JointConstSharedPtr joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
      joint_num++;
      float lower, upper;
      int hasLimits;
      if (joint->type != urdf::Joint::CONTINUOUS) {
        if (joint->safety) {
          lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
          upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
        } else {
          lower = joint->limits->lower;
          upper = joint->limits->upper;
        }
        hasLimits = 1;
      } else {
        hasLimits = 0;
      }
      if (hasLimits) {
        lb(joint_num - 1) = lower + mergin;
        ub(joint_num - 1) = upper - mergin;
      } else {
        lb(joint_num - 1) = std::numeric_limits<float>::lowest();
        ub(joint_num - 1) = std::numeric_limits<float>::max();
      }
      // ROS_DEBUG_STREAM("IK Using joint " << joint->name << " " << lb(joint_num - 1) << " " << ub(joint_num - 1));

      float vlimit;
      int hasVLimits;
      if (joint->type != urdf::Joint::CONTINUOUS) {
        vlimit = joint->limits->velocity;
        hasVLimits = 1;
      } else {
        hasVLimits = 0;
      }
      if (hasVLimits) {
        vb(joint_num - 1) = vlimit;
      } else {
        vb(joint_num - 1) = std::numeric_limits<float>::max();
      }
      // ROS_DEBUG_STREAM("IK Using joint " << joint->name << " " << vb(joint_num - 1));
    }
  }
}
};  // namespace ModelUtility

#endif  // MODEL_UTILITY_H