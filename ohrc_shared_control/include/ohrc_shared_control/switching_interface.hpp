#ifndef SWITCHING_INTERFACE_HPP
#define SWITCHING_INTERFACE_HPP

#include "ohrc_control/interface.hpp"
#include <std_srvs/Empty.h>

template <class T1, class T2>
class SwitchingInterface: virtual public T1, virtual public T2{

ros::ServiceServer srv;
int cur_interface = 0;

protected:
  void switchInterface(){
    cur_interface = (cur_interface + 1) % 2;
    ROS_INFO_STREAM( "cur_interface: " << cur_interface);

    if (cur_interface==0)
       T1::resetInterface();
       else
       T2::resetInterface();
  }

  bool srvSwitchInterface(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    ROS_WARN_STREAM("/switch_interface was called");
    switchInterface();
    return true;
  }

public:
  SwitchingInterface(const std::shared_ptr<CartController> controller) : T1(controller), T2(controller), Interface(controller){
    srv = T1::n.advertiseService("/switch_interface", &SwitchingInterface::srvSwitchInterface, this);
  }

  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override{

    if (cur_interface == 0)
      T1::updateTargetPose(pose, twist);

    else 
      T2::updateTargetPose(pose, twist);

  }

  void initInterface() override {
    T1::initInterface();
    T2::initInterface();
  };

  void resetInterface() override {
    T1::resetInterface();
    T2::resetInterface();
  };
};




#endif //SWITCHING_INTERFACE_HPP