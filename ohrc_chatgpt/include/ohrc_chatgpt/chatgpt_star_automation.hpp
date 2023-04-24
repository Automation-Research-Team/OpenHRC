#include <Eigen/Geometry>
#include <cmath>

#include "ohrc_control/interface.hpp"

class MoveEndEffector : public Interface {
public:
  MoveEndEffector(std::shared_ptr<CartController> controller) : Interface(controller) {
  }

  void initInterface() override {
    startTime = ros::Time::now();
    loopCount = 0;
    updateStartTime = true;
    controller->startOperation();
  }

  void resetInterface() override {
    startTime = ros::Time::now();
    loopCount = 0;
    updateStartTime = true;
    taskState = TaskState::Initial;
  }

  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) override {
    if (taskState == TaskState::Initial) {
      T_init = controller->getT_init();
      initInterface();
      taskState = TaskState::OnGoing;
    }

    if (updateStartTime) {
      startTime = ros::Time::now();
      updateStartTime = false;
    }

    double elapsedTime = (ros::Time::now() - startTime).toSec();

    if (elapsedTime >= 20.0) {
      elapsedTime = 20.0;
      updateStartTime = true;
      loopCount++;
      std::cout << "loop finished" << std::endl;
    }

    double t = elapsedTime / 20.0;

    Eigen::Affine3d endEffectorPosition = starTrajectory(T_init, t);
    pose = KDL::Frame(KDL::Rotation(endEffectorPosition.rotation()(0, 0), endEffectorPosition.rotation()(0, 1), endEffectorPosition.rotation()(0, 2),
                                    endEffectorPosition.rotation()(1, 0), endEffectorPosition.rotation()(1, 1), endEffectorPosition.rotation()(1, 2),
                                    endEffectorPosition.rotation()(2, 0), endEffectorPosition.rotation()(2, 1), endEffectorPosition.rotation()(2, 2)),
                      KDL::Vector(endEffectorPosition.translation()(0), endEffectorPosition.translation()(1), endEffectorPosition.translation()(2)));

    twist = calculateTwist(T_init, t);

    if (elapsedTime >= 20.0 && loopCount > 0) {
      ros::Duration(3.0).sleep();
      resetInterface();
    }
  }

private:
  Eigen::Affine3d T_init;
  ros::Time startTime;
  int loopCount;
  bool updateStartTime;

  Eigen::Affine3d starTrajectory(const Eigen::Affine3d& T_init, double t) {
    double x = 0.1 * (2 * std::sin(4 * M_PI * t) * std::cos(2 * M_PI * t));
    double y = 0.1 * (2 * std::sin(4 * M_PI * t) * std::sin(2 * M_PI * t));

    Eigen::Affine3d T_star(Eigen::Translation3d(x, y, 0.0));
    return T_init * T_star;
  }

  KDL::Twist calculateTwist(const Eigen::Affine3d& T_init, double t) {
    double x_dot = 0.1 * (8 * M_PI * std::cos(4 * M_PI * t) * std::cos(2 * M_PI * t) - 4 * M_PI * std::sin(4 * M_PI * t) * std::sin(2 * M_PI * t));
    double y_dot = 0.1 * (8 * M_PI * std::cos(4 * M_PI * t) * std::sin(2 * M_PI * t) + 4 * M_PI * std::sin(4 * M_PI * t) * std::cos(2 * M_PI * t));
    KDL::Vector linearTwist(x_dot, y_dot, 0.0);
    KDL::Vector angularTwist(0.0, 0.0, 0.0);

    return KDL::Twist(linearTwist, angularTwist);
  }
};
