#ifndef RCLCPP_UTILITY_HPP
#define RCLCPP_UTILITY_HPP

#include <rclcpp/rclcpp.hpp>

#include "magic_enum.hpp"

namespace RclcppUtility {

template <typename T>
T print(const T& value) {
  return value;  // vectorでない場合
}

template <typename T>
std::string print(const std::vector<T>& vec) {
  std::stringstream ss;
  // std::string str = "[";
  ss << "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    ss << vec[i];
    if (i < vec.size() - 1) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
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

template <typename T>
inline bool declare_and_get_parameter_enum(std::shared_ptr<rclcpp::Node> node, std::string param_name, T default_enum, T& param_enum, bool verbose = true) {
  std::string param_value;
  RclcppUtility::declare_and_get_parameter(node, param_name, std::string(""), param_value, false);

  param_enum = magic_enum::enum_cast<T>(param_value).value_or(T::None);
  if (param_enum == T::None) {
    param_enum = default_enum;
    constexpr auto names = magic_enum::enum_names<T>();
    if (verbose)
      RCLCPP_WARN_STREAM(node->get_logger(), "Got paramter " << param_name << ": " << param_value << " is not in " << magic_enum::enum_type_name<decltype(param_enum)>() << " "
                                                             << RclcppUtility::print(std::vector<std::string>(names.begin(), names.end() - 1))
                                                             << ". Using default value: " << magic_enum::enum_name(default_enum));
    return false;
  } else if (verbose)
    RCLCPP_INFO_STREAM(node->get_logger(), "Parameter " << param_name << ": " << magic_enum::enum_name(param_enum));

  return true;
}
};  // namespace RclcppUtility

#endif  // RCLCPP_UTILITY_HPP