#ifndef RCLCPP_UTILITY_HPP
#define RCLCPP_UTILITY_HPP

#include <rclcpp/rclcpp.hpp>

namespace RclcppUtility {

template <typename T>
T print(const T& value) {
  return value;  // vectorでない場合
}

template <typename T>
std::string print(const std::vector<T>& vec) {
  std::string str = "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    str += vec[i];
    if (i < vec.size() - 1) {
      str += ", ";
    }
  }
  str += "]";
  return str;
}

template <typename T>
inline bool declare_and_get_parameter(std::shared_ptr<rclcpp::Node> node, std::string param_name, T default_value, T& param_value, bool verbose = true) {
  node->declare_parameter(param_name, default_value);
  if (!node->get_parameter(param_name, param_value)) {
    if (verbose)
      RCLCPP_WARN_STREAM(node->get_logger(), "Failed to get parameter: " << param_name << ". Using default value: " << RclcppUtility::print(default_value));
    return false;
  } else if (verbose)
    RCLCPP_INFO_STREAM(node->get_logger(), "Parameter " << param_name << ": " << RclcppUtility::print(param_value));

  return true;
}

};  // namespace RclcppUtility

#endif  // RCLCPP_UTILITY_HPP