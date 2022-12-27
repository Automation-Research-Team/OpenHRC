#include "ohrc_common/transform_utility.h"

TransformUtility::TransformUtility() : tfListener(tfBuffer) {
  // get transform from TF
  // try {
  //   this->Thandle_fp = this->getTransform("ics_walker/base_footprint", "ics_walker/handle_link", ros::Time(0), ros::Duration(1.0));
  //   T_h_fp = Thandle_fp;  // rename

  //   this->Thandle_ft = this->getTransform("ics_walker/handle_link", "ics_walker/ft_sensor_link", ros::Time(0), ros::Duration(1.0));

  //   this->Thandle_lidar = this->getTransform("ics_walker/lrf_link", "ics_walker/handle_link", ros::Time(0), ros::Duration(1.0));

  //   this->T_l_fp = this->getTransform("ics_walker/base_footprint", "ics_walker/lrf_link", ros::Time(0), ros::Duration(1.0));

  //   this->T_c_fp = this->getTransform("ics_walker/base_footprint", "ics_walker/camera_depth_optical_frame", ros::Time(0), ros::Duration(1.0));

  //   this->T_color_depth = this->getTransform("ics_walker/camera_depth_optical_frame", "ics_walker/camera_color_optical_frame", ros::Time(0), ros::Duration(1.0));
  // } catch (tf2::TransformException& ex) {
  //   ROS_ERROR("%s", ex.what());
  //   // ros::shutdown();
  // }
}

// MatrixXd TransformUtility::transformFT_ft2handle(VectorXd ft_sensor) {
//   return transformFT(ft_sensor, Thandle_ft);
// }

// MatrixXd TransformUtility::transformV_handle2fp(VectorXd targetV) {
//   return transformVel(targetV, this->Thandle_fp.inverse());
// }

// MatrixXd TransformUtility::transformV_fp2handle(VectorXd targetV) {
//   return transformVel(targetV, this->Thandle_fp);
// }

Eigen::Affine3d TransformUtility::getTransform(const std::string& target, const std::string& source, const ros::Time& time, ros::Duration timeout) {
  return tf2::transformToEigen(tfBuffer.lookupTransform(target, source, time, timeout));
}
