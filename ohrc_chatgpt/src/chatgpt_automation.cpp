#include "ohrc_control/single_interface.hpp"

////////////////////////////////////////
// move_end_effector.hpp
#include <cmath>
#include <iostream>
#include <ohrc_control/interface.hpp>

class MoveEndEffector : public Interface {
 private:
  enum class MoveState { Start, Move1, Move2, Move3, Move4, Wait };

  MoveState moveState;
  double side_length = 0.1;    // 10 cm
  double move_duration = 3.0;  // 3 seconds
  double wait_duration = 3.0;  // 3 seconds
  double elapsed_time = 0.0;

  Eigen::Affine3d T_init;

 public:
  MoveEndEffector(std::shared_ptr<CartController> controller)
      : Interface(controller) {
    moveState = MoveState::Start;
  }

  void initInterface() override {
    T_init = controller->getT_init();
    controller->startOperation();
  }

  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override {
    if (taskState == TaskState::Initial) {
      initInterface();
      taskState = TaskState::OnGoing;
    }

    double progress = fmod(elapsed_time, move_duration) / move_duration;
    double delta = side_length * progress;
    double twist_speed = side_length / move_duration;

    switch (moveState) {
      case MoveState::Start:
        moveState = MoveState::Move1;
        elapsed_time = 0.0;
        break;
      case MoveState::Move1:
        pose.p.x(T_init.translation().x() + delta);
        pose.p.z(T_init.translation().z());
        twist.vel.x(twist_speed);
        twist.vel.z(0.0);
        break;
      case MoveState::Move2:
        pose.p.x(T_init.translation().x() + side_length);
        pose.p.z(T_init.translation().z() + delta);
        twist.vel.x(0.0);
        twist.vel.z(twist_speed);
        break;
      case MoveState::Move3:
        pose.p.x(T_init.translation().x() + side_length - delta);
        pose.p.z(T_init.translation().z() + side_length);
        twist.vel.x(-twist_speed);
        twist.vel.z(0.0);
        break;
      case MoveState::Move4:
        pose.p.x(T_init.translation().x());
        pose.p.z(T_init.translation().z() + side_length - delta);
        twist.vel.x(0.0);
        twist.vel.z(-twist_speed);
        break;
      case MoveState::Wait:
        pose = T_init;
        twist.Zero();
        break;
    }

    pose.M = T_init.rotation();
    elapsed_time += dt;

    if (elapsed_time >= move_duration) {
      elapsed_time = 0.0;
      switch (moveState) {
        case MoveState::Move1:
          moveState = MoveState::Move2;
          break;
        case MoveState::Move2:
          moveState = MoveState::Move3;
          break;
        case MoveState::Move3:
          moveState = MoveState::Move4;
          break;
        case MoveState::Move4:
          moveState = MoveState::Wait;
          break;
        case MoveState::Wait:
          std::cout << "loop finished" << std::endl;
          moveState = MoveState::Move1;
          break;
      }
    }
  }

  void resetInterface() override {
    taskState = TaskState::Initial;
    moveState = MoveState::Start;
    elapsed_time = 0.0;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "marker_twist_topic_teleoperation");

  SingleInterface<MoveEndEffector> interface;
  if (interface.control() < 0) ROS_ERROR("End by some fails");
}
