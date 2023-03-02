#ifndef OHRC_CONTROL_HPP
#define OHRC_CONTROL_HPP

namespace ohrc_control {
enum class SolverType { Trac_IK, KDL, MyIK, None };
enum class ControllerType { Position, Velocity, Torque, None };
enum class PublisherType { Position, Velocity, Torque, Trajectory, TrajectoryAction, None };

enum class TaskState {
  Initial,
  OnGoing,
  Success,
  Fail,
};

}  // namespace ohrc_control

#endif  // OHRC_CONTROL_HPP