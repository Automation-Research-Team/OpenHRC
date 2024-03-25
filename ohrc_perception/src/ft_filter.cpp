#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/EmptyResponse.h>

#include <Eigen/Geometry>
#include <mutex>

#include "ohrc_common/geometry_msgs_utility/geometry_msgs_utility.h"
#include "ohrc_msgs/Contact.h"

std::unique_ptr<geometry_msgs_utility::WrenchStamped> forceLpf, forceLpf_;
ros::Publisher pub, pub_cnt;

unsigned int count = 1;
unsigned int nMax = 1000;
Eigen::VectorXd offset = Eigen::VectorXd::Zero(6);

std::mutex mtx;
geometry_msgs::WrenchStamped raw_;

void cbForce(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  // geometry_msgs::WrenchStamped raw_dz, filtered;

  std::lock_guard<std::mutex> lock(mtx);
  raw_ = *msg;
  // if (n < nMax) {
  //   filtered.header = msg->header;
  //   pub.publish(filtered);
  //   offset = (offset * (n - 1) + tf2::fromMsg(msg->wrench)) / n;
  //   n++;
  //   return;
  // }

  // raw_dz.header = msg->header;
  // tf2::toMsg(tf2::fromMsg(msg->wrench) - offset, raw_dz.wrench);
  // forceLpf->deadZone_LPF(raw_dz, filtered);

  // pub.publish(filtered);
}

bool reset_offset(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  std::lock_guard<std::mutex> lock(mtx);
  count = 1;
  offset = Eigen::VectorXd::Zero(6);
  ROS_INFO_STREAM("Resetting ft sensor offset...");
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ft_sensor_filter");

  ros::NodeHandle n("~"), nh;

  geometry_msgs_utility::Wrench::paramLPF paramLpf;
  geometry_msgs_utility::paramDeadZone paramDeadZone;

  std::string topic_name_raw;
  if (!n.param("topic_name/raw", topic_name_raw, std::string("/force")))
    ROS_ERROR("Failed to get ft sensor rate");

  std::string topic_name_filtered;
  if (!n.param("topic_name/filtered", topic_name_filtered, std::string("/force_filtered")))
    ROS_ERROR("Failed to get ft sensor rate");

  if (!n.param("rate", paramLpf.sampling_freq, 1000.0))
    ROS_ERROR("Failed to get ft sensor rate");
  nMax = paramLpf.sampling_freq * 3.0;  // 1 second

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
  forceLpf_.reset(new geometry_msgs_utility::WrenchStamped(paramLpf, paramDeadZone));

  ros::ServiceServer service = n.advertiseService("reset_offset", reset_offset);

  ros::Subscriber sub = nh.subscribe(topic_name_raw, 1, cbForce);
  pub = nh.advertise<geometry_msgs::WrenchStamped>(topic_name_filtered, 2);
  pub_cnt = nh.advertise<ohrc_msgs::Contact>("contact", 2);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Rate r(paramLpf.sampling_freq);
  while (ros::ok()) {
    geometry_msgs::WrenchStamped raw, raw_dz, filtered;
    {
      std::lock_guard<std::mutex> lock(mtx);
      raw = raw_;
    }
    if (count < nMax) {
      offset = (offset * (count - 1) + tf2::fromMsg(raw.wrench)) / count;
      count++;
      r.sleep();
      continue;
    }

    raw_dz.header = raw.header;

    tf2::toMsg(tf2::fromMsg(raw.wrench) - offset, raw_dz.wrench);
    forceLpf_->LPF(raw_dz, raw_dz);
    forceLpf->deadZone_LPF(raw_dz, filtered);

    pub.publish(filtered);

    ohrc_msgs::Contact cnt;
    cnt.f_norm = tf2::fromMsg(filtered.wrench).norm();

    if (cnt.f_norm > 10.0)
      cnt.contact = true;

    pub_cnt.publish(cnt);

    r.sleep();
  }

  // ros::spin();

  return 0;
}
