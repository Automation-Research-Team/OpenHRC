#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>

#include <mutex>
#include <thread>
ros::Time t0;
ros::Publisher chatter_pub;
int _trigger;

std::mutex mtx;

void ThreadFunc2() {
  std_msgs::Int8 trigger;
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    {
      std::lock_guard<std::mutex> lock(mtx);
      trigger.data = _trigger;
    }
    chatter_pub.publish(trigger);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "triger");
  _trigger = 0;
  ros::NodeHandle n;

  chatter_pub = n.advertise<std_msgs::Int8>("trigger", 1000);

  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("add_two_ints");
  std_srvs::Empty srv;

  ros::Rate loop_rate(10);

  int count = 0;
  t0 = ros::Time::now();

  // std::thread th1(ThreadFunc);
  std::thread th2(ThreadFunc2);

  while (ros::ok()) {
    // std::cout << "\r" << std::setprecision(3) << (ros::Time::now() - t0).toSec() << "                                             " << std::flush;
    std::string s;
    if (std::getline(std::cin, s)) {
      std::lock_guard<std::mutex> lock(mtx);
      if (s == "a") {
        t0 = ros::Time::now();
        _trigger++;
        ROS_INFO_STREAM("Start Logging!");
      } else if (s == "s") {
        _trigger = 0;
        ROS_INFO_STREAM("Stop Logging!");
      } else if (s == "r") {
        client.call(srv);
        ROS_INFO_STREAM("Sent reset service)");
      } else if (s == "q") {
        ROS_INFO_STREAM("Quit");
        ros::shutdown();
      }
    }
    // std::cout << "ll" << std::endl;
    // loop_rate.sleep();
  }

  // th1.join();
  th2.join();

  return 0;
}