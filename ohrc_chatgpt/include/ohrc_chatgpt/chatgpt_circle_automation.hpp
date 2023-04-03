#include <eigen_conversions/eigen_kdl.h>

#include <cmath>
#include <iostream>
#include <memory>

#include "ohrc_control/interface.hpp"

class MoveEndEffector : public Interface {
public:
  MoveEndEffector(std::shared_ptr<CartController> controller) : Interface(controller) {
  }

  virtual void initInterface() override {
    Eigen::Affine3d T_init_eigen = controller->getT_init();
    tf::transformEigenToKDL(T_init_eigen, T_init);
    controller->startOperation();
    time_elapsed = 0;
    loop_finished = false;
  }

  virtual void resetInterface() override {
    taskState = TaskState::Initial;
    time_elapsed = 0;
    loop_finished = false;
  }

  virtual void updateTargetPose(KDL::Frame &pose, KDL::Twist &twist) override {
    if (taskState == TaskState::Initial) {
      initInterface();
      taskState = TaskState::OnGoing;
    }

    if (loop_finished) {
      stay_time += dt;
      if (stay_time >= 3.0) {
        stay_time = 0;
        loop_finished = false;
        time_elapsed = 0;
        std::cout << "loop finished" << std::endl;
      }
    } else {
      double radius = 0.05;            // half of the diameter (10cm)
      double omega = 2 * M_PI / 10.0;  // one cycle in 10 seconds
      double x_offset = radius * sin(omega * time_elapsed);
      double z_offset = radius * (1 - cos(omega * time_elapsed));

      pose = T_init * KDL::Frame(KDL::Vector(x_offset, 0, z_offset));
      twist = KDL::Twist(KDL::Vector(omega * radius * cos(omega * time_elapsed), 0, omega * radius * sin(omega * time_elapsed)), KDL::Vector::Zero());

      time_elapsed += dt;
      if (time_elapsed >= 10.0) {
        loop_finished = true;
      }
    }
  }

private:
  KDL::Frame T_init;
  double time_elapsed = 0;
  double stay_time = 0;
  bool loop_finished = false;
};
