#include "keyboard/Key.h"
#include "omega_haptic_device/Omega.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

std::vector<ros::Publisher> pub;
uint16_t next_mode = 1;
uint16_t mode = 1;

void chatterCallback(const ohrc_msgs::State::ConstPtr& msg) {
  if (!msg->gripper.button)
    mode = next_mode;

  int robot = 1;
  if (mode == keyboard::Key::KEY_1)
    robot = 1;
  else if (mode == keyboard::Key::KEY_2)
    robot = 2;
  else if (mode == keyboard::Key::KEY_p)
    robot = 99;
  else if (mode == keyboard::Key::KEY_c)
    robot = 999;

  if (robot == 99)
    for (int i = 0; i < pub.size() - 1; i++)
      pub[i].publish(*msg);
  else if (robot == 999)
    pub[pub.size() - 1].publish(*msg);
  else
    pub[robot - 1].publish(*msg);
}

void cmdCallback(const keyboard::Key::ConstPtr& msg) {
  next_mode = msg->code;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "listener");

  ros::NodeHandle n("~");

  std::string omega_type;
  n.getParam("omega_type", omega_type);

  std::vector<std::string> follower_list;
  n.getParam("follower_list", follower_list);

  std::cout << follower_list[0] << std::endl;
  ros::Subscriber sub = n.subscribe("/omega_driver/" + omega_type + "/state", 2, chatterCallback);
  ros::Subscriber sub2 = n.subscribe("cmd", 2, cmdCallback);

  pub.resize(follower_list.size());
  for (int i = 0; i < follower_list.size(); i++)
    pub[i] = n.advertise<ohrc_msgs::State>("/omega_driver/" + follower_list[i] + "/state", 2);

  pub.push_back(n.advertise<ohrc_msgs::State>("/omega_driver/cooperation/state", 2));

  ros::spin();

  return 0;
}