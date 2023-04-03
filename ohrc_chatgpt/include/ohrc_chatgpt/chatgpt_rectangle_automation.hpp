#include <Eigen/Geometry>

#include "ohrc_control/interface.hpp"

class MoveEndEffector : public Interface {
public:
  MoveEndEffector(std::shared_ptr<CartController> controller) : Interface(controller) {
  }

  void initInterface() override {
    t_init = controller->getT_init();
    cur_state = State::Start;
    controller->startOperation();
  }

  void resetInterface() override {
    cur_state = State::Start;
  }

  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override {
    if (cur_state == State::Start) {
      t_init = controller->getT_init();
      t_start = ros::Time::now();
      cur_state = State::Moving;
    }

    double t_elapsed = (ros::Time::now() - t_start).toSec();
    Eigen::Affine3d cur_pose = t_init;

    switch (cur_state) {
      case State::Moving: {
        int phase = static_cast<int>(t_elapsed / 3.0) % 4;

        double t_phase = fmod(t_elapsed, 3.0);
        double progress = t_phase / 3.0;

        double x = 0.0, z = 0.0;

        switch (phase) {
          case 0:
            x = 0.0 + progress * 0.1;
            z = 0.0;
            break;
          case 1:
            x = 0.1;
            z = 0.0 + progress * 0.1;
            break;
          case 2:
            x = 0.1 - progress * 0.1;
            z = 0.1;
            break;
          case 3:
            x = 0.0;
            z = 0.1 - progress * 0.1;
            break;
        }

        cur_pose.translate(Eigen::Vector3d(x, 0.0, z));
        pose = KDL::Frame(
            KDL::Rotation::Quaternion(cur_pose.rotation().coeff(0, 0), cur_pose.rotation().coeff(1, 0), cur_pose.rotation().coeff(2, 0), cur_pose.rotation().coeff(3, 0)),
            KDL::Vector(cur_pose.translation().x(), cur_pose.translation().y(), cur_pose.translation().z()));

        twist =
            KDL::Twist(KDL::Vector((phase == 0 ? 1 : (phase == 2 ? -1 : 0)) * 0.1 / 3.0, 0.0, (phase == 1 ? 1 : (phase == 3 ? -1 : 0)) * 0.1 / 3.0), KDL::Vector(0.0, 0.0, 0.0));

        if (fmod(t_elapsed, 12.0) >= 12.0 - dt) {
          cur_state = State::Rest;
          t_start = ros::Time::now();
        }
      } break;

      case State::Rest: {
        pose = KDL::Frame(
            KDL::Rotation::Quaternion(cur_pose.rotation().coeff(0, 0), cur_pose.rotation().coeff(1, 0), cur_pose.rotation().coeff(2, 0), cur_pose.rotation().coeff(3, 0)),
            KDL::Vector(cur_pose.translation().x(), cur_pose.translation().y(), cur_pose.translation().z()));
        twist = KDL::Twist::Zero();
        if (t_elapsed >= 3.0) {
          cur_state = State::Moving;
          t_start = ros::Time::now();
          std::cout << "loop finished" << std::endl;

          // Reset initial position for the next loop
          t_init = controller->getT_init();
        }
      } break;
    }
  }

private:
  enum class State {
    Start,
    Moving,
    Rest,
  };

  State cur_state;
  Eigen::Affine3d t_init;
  ros::Time t_start;
};