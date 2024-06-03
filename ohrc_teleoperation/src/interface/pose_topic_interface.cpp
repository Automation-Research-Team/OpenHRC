#include "ohrc_teleoperation/pose_topic_interface.hpp"

void PoseTopicInterface::initInterface() {
  setSubscriber();

  Affine3d T_state_base = controller->getTransform_base(this->stateFrameId);
  R = T_state_base.rotation();

  bool diablePoseFeedback;
  n.param("diable_pose_feedback", diablePoseFeedback, false);

  controller->enablePoseFeedback();  // tentative
  // if (diablePoseFeedback) {
  //   ROS_WARN_STREAM("Pose feedback is disabled");
  //   controller->disablePoseFeedback();
  // }

  controller->updatePosFilterCutoff(10.0);
}

void PoseTopicInterface::setSubscriber() {
  getTopicAndFrameName("/cmd_pose", "user_frame");
  subPose = n.subscribe<geometry_msgs::Pose>(stateTopicName, 1, &PoseTopicInterface::cbPose, this, th);
}

void PoseTopicInterface::cbPose(const geometry_msgs::Pose::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_topic);
  _pose = *msg;
  _flagTopic = true;
}

void PoseTopicInterface::updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) {
  geometry_msgs::Pose markerPose;
  double markerDt;
  {
    std::lock_guard<std::mutex> lock(mtx_topic);
    if (!_flagTopic) {
      return;
    }
    markerPose = _pose;
    controller->enableOperation();
  }

  Affine3d T_topic;
  tf2::fromMsg(markerPose, T_topic);
  Affine3d T_base = Translation3d(R * T_topic.translation()) * (R * T_topic.rotation());

  static Affine3d initT = T_base;

  // if (prevPoses.p.data[0] == 0.0 && prevPoses.p.data[1] == 0.0 && prevPoses.p.data[2] == 0.0)  // initilize
  // prevPoses = pose;

  Vector3d pos = T_base.translation() - initT.translation() + controller->getT_init().translation();

  tf::vectorEigenToKDL(pos, pose.p);
  //   tf::vectorEigenToKDL(controller->getT_init().rotation(), pose.rot);

  // controller->getVelocity(pose, prevPoses, dt, twist);  // TODO: get this velocity in periodic loop using Kalman filter

  // prevPoses = pose;
}

void PoseTopicInterface::resetInterface() {
  ROS_WARN_STREAM("Reset marker position");
  _flagTopic = false;
}