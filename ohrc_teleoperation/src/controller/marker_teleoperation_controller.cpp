#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include <urdf/model.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include "ohrc_control/single_interface.hpp"
#include "ohrc_teleoperation/marker_interface.hpp"

class MarkerTeleoperationController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
public:
  bool init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *
   * \param command
   */
  void setCommand(double pos_target);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *        Also supports a target velocity
   *
   * \param pos_target - position setpoint
   * \param vel_target - velocity setpoint
   */
  void setCommand(double pos_target, double vel_target);

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void starting(const ros::Time &time);

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void update(const ros::Time &time, const ros::Duration &period);
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "marker_teleoperation");
  SingleInterface<MarkerInterface> interface;
  if (interface.control() < 0)
    ROS_ERROR("End by some fails");
}