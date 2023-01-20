#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <mutex>
#include <thread>
ros::Time t0;
ros::Publisher chatter_pub;
int _trigger;

bool begin_write = false;

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

void chatterCallback(const std_msgs::String& msg) {
  std::lock_guard<std::mutex> lock(mtx);
  begin_write = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "triger");
  _trigger = 0;
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/begin_write", 10, chatterCallback);
  chatter_pub = n.advertise<std_msgs::Int8>("trigger", 1000);

  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/reset");
  std_srvs::Empty srv;

  ros::Rate loop_rate(10);
  t0 = ros::Time::now();

  int count = 1;
  while (ros::ok()) {
    {
      std::lock_guard<std::mutex> lock(mtx);
      if (begin_write)
        break;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO_STREAM("Start trigger");

  // std::thread th1(ThreadFunc);
  std::thread th2(ThreadFunc2);

  std::string s;

  while (ros::ok()) {
    ROS_INFO_STREAM("start_log (a), stop_log (s), reset (r), Quit (q)");
    if (std::getline(std::cin, s)) {
      std::lock_guard<std::mutex> lock(mtx);
      if (s == "a") {
        t0 = ros::Time::now();
        _trigger++;
        ROS_INFO_STREAM("Start Logging! Count: " << count);
      } else if (s == "s") {
        if (_trigger > 0) {
          _trigger = 0;
          ROS_INFO_STREAM("Stop Logging! Count: " << count << ", Recording Time: " << (ros::Time::now() - t0).toSec());
          count++;
        }
      } else if (s == "r") {
        if (!client.call(srv))
          ROS_WARN_STREAM("Failed to call reset service.");
        ROS_INFO_STREAM("Sent reset service");
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