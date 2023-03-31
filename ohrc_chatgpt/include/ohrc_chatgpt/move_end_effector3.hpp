#include <cmath>
#include <memory>
#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <Eigen/Geometry>

#include "ohrc_control/interface.hpp"

class MoveEndEffector : public Interface {
public:
  MoveEndEffector(std::shared_ptr<CartController> controller)
      : Interface(controller), startTime(0.0), cycleTime(30.0), stayTime(3.0), width(0.2), height(0.2) {}

  virtual void updateTargetPose(KDL::Frame &pose, KDL::Twist &twist) override {
    if (taskState == TaskState::Initial) {
      T_init = KDL::Frame(Eigen::Matrix<double, 4, 4, Eigen::RowMajor>(controller->getT_init().matrix()));
      pose = T_init;
      twist = KDL::Twist::Zero();

      controller->startOperation();
      taskState = TaskState::OnGoing;

      startTime = ros::Time::now().toSec();
    } else if (taskState == TaskState::OnGoing) {
      double elapsedTime = ros::Time::now().toSec() - startTime;
      double progress = fmod(elapsedTime, cycleTime + stayTime);

      if (progress < cycleTime) {
        double t = progress / cycleTime;
        double x = T_init.p.x();
        double y = T_init.p.y() + width * (1.0 - cos(2.0 * M_PI * t)) * cos(M_PI * t);
        double z = T_init.p.z() + height * (1.0 - cos(2.0 * M_PI * t)) * sin(M_PI * t);

        pose.p = KDL::Vector(x, y, z);
        pose.M = T_init.M;

        double dt = 1.0 / controller->dt;
        double dy_dt = width * (2.0 * M_PI * sin(2.0 * M_PI * t) * cos(M_PI * t) - M_PI * (1.0 - cos(2.0 * M_PI * t)) * sin(M_PI * t));
        double dz_dt = height * (2.0 * M_PI * sin(2.0 * M_PI * t) * sin(M_PI * t) + M_PI * (1.0 - cos(2.0 * M_PI * t)) * cos(M_PI * t));

        twist = KDL::Twist(KDL::Vector(0.0, dy_dt, dz_dt) * dt, KDL::Vector::Zero());
      } else {
        pose = T_init;
        twist = KDL::Twist::Zero();

        if (progress >= cycleTime + stayTime - controller->dt) {
          ROS_INFO("loop finished");
        }
      }
    }
  }

private:
  KDL::Frame T_init;
  double startTime;
  double cycleTime;
  double stayTime;
  double width;
  double height;
};
