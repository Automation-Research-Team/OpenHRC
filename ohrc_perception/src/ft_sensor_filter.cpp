#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/EmptyResponse.h>

#include <Eigen/Geometry>
#include <mutex>

#include "toroboarm_common/geometry_msgs_utility/geometry_msgs_utility.h"

std::unique_ptr<geometry_msgs_utility::WrenchStamped> forceLpf;
ros::Publisher pub;

unsigned int n = 1;
unsigned int nMax = 1000;
Eigen::VectorXd offset = Eigen::VectorXd::Zero(6);

std::mutex mtx;

void cbForce(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  geometry_msgs::WrenchStamped raw_dz, filtered;

  std::lock_guard<std::mutex> lock(mtx);
  if (n < nMax) {
    filtered.header = msg->header;
    pub.publish(filtered);
    offset = (offset * (n - 1) + tf2::fromMsg(msg->wrench)) / n;
    n++;
    return;
  }

  raw_dz.header = msg->header;
  tf2::toMsg(tf2::fromMsg(msg->wrench) - offset, raw_dz.wrench);
  forceLpf->deadZone_LPF(raw_dz, filtered);

  pub.publish(filtered);
}

bool reset_offset(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  std::lock_guard<std::mutex> lock(mtx);
  n = 1;
  offset = Eigen::VectorXd::Zero(6);
  ROS_INFO_STREAM("Resetting ft sensor offset...");
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ft_sensor_filter");

  ros::NodeHandle n("~"), nh;

  geometry_msgs_utility::Wrench::paramLPF paramLpf;
  geometry_msgs_utility::paramDeadZone paramDeadZone;
  if (!nh.param("dynpick_driver/rate", paramLpf.sampling_freq, 1000.0))
    ROS_ERROR("Failed to get ft sensor rate");
  nMax = paramLpf.sampling_freq * 1;  // 1 second

  if (!n.param("cutoff_freq", paramLpf.cutoff_freq, 200.0))
    ROS_ERROR("Failed to get cutoff_freq setting");

  if (!n.param("lpf_order", paramLpf.order, 2))
    ROS_ERROR("Failed to get lpf_order setting");

  if (!n.param("deadzone/force/lower", paramDeadZone.force_lower, -1.0))
    ROS_ERROR("Failed to get deadzone/force/lower setting");

  if (!n.param("deadzone/force/upper", paramDeadZone.force_upper, 1.0))
    ROS_ERROR("Failed to get deadzone/force/upper setting");

  if (!n.param("deadzone/torque/lower", paramDeadZone.torque_lower, -1.0))
    ROS_ERROR("Failed to get deadzone/torque/lower setting");

  if (!n.param("deadzone/torque/upper", paramDeadZone.torque_upper, 1.0))
    ROS_ERROR("Failed to get deadzone/torque/upper setting");

  forceLpf.reset(new geometry_msgs_utility::WrenchStamped(paramLpf, paramDeadZone));

  ros::ServiceServer service = nh.advertiseService("ft_sensor/reset_offset", reset_offset);

  ros::Subscriber sub = nh.subscribe("ft_sensor/raw", 2, cbForce);
  pub = nh.advertise<geometry_msgs::WrenchStamped>("ft_sensor/filtered", 2);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  // ros::spin();

  return 0;
}
