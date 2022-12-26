#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <mutex>

class PassivityObserver {
  std::mutex mtx;
  ros::NodeHandle nh;
  ros::Subscriber subSlaveEnergy, subMasterEnergy;

  void cbSlaveEnergy(const std_msgs::Float32::ConstPtr &msg);
  void cbMasterEnergy(const std_msgs::Float32::ConstPtr &msg);

  double masterEnergy = 0.0, slaveEnergy = 0.0;
  ros::AsyncSpinner spinner;

public:
  PassivityObserver();
  int main();
};

PassivityObserver::PassivityObserver() : spinner(2) {
  ros::TransportHints th = ros::TransportHints().tcpNoDelay(true);
  subSlaveEnergy = nh.subscribe<std_msgs::Float32>("/passivity_observer/slave/energy", 2, &PassivityObserver::cbSlaveEnergy, this, th);
  subMasterEnergy = nh.subscribe<std_msgs::Float32>("/passivity_observer/master/energy", 2, &PassivityObserver::cbMasterEnergy, this, th);
}

void PassivityObserver::cbSlaveEnergy(const std_msgs::Float32::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mtx);
  slaveEnergy = msg->data;
}

void PassivityObserver::cbMasterEnergy(const std_msgs::Float32::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mtx);
  masterEnergy = msg->data;
}

int PassivityObserver::main() {
  spinner.start();

  ros::Rate r(500.0);

  while (ros::ok()) {
    std::lock_guard<std::mutex> lock(mtx);
    double energy = masterEnergy - slaveEnergy;
    std::cout << energy << std::endl;

    r.sleep();
  }

  return 1;
}

//////////////////////////////////////////////
int main(int argc, char **argv) {
  ros::init(argc, argv, "omega_force_driver");

  PassivityObserver PassivityObserver;
  PassivityObserver.main();

  return 0;
}