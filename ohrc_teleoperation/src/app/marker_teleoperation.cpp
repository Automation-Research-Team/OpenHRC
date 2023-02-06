
#include "ohrc_control/multi_cart_controller.hpp"
#include "ohrc_teleoperation/marker_interface.hpp"

class MarkerInterfaceApp : virtual public MultiCartController {
  std::vector<std::shared_ptr<MarkerInterface>> markerInterface;

  void updateTargetPose(KDL::Frame& pose, KDL::Twist& twist, std::shared_ptr<CartController> controller) override {
    markerInterface[controller->getIndex()]->updateTargetPose(pose, twist);
  }

  void resetInterface(std::shared_ptr<CartController> controller) override {
    markerInterface[controller->getIndex()]->resetInterface();
  }

public:
  MarkerInterfaceApp() {
    markerInterface.resize(nRobot);

    for (int i = 0; i < nRobot; i++)
      markerInterface[i].reset(new MarkerInterface(cartControllers[i]));
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "marker_teleoperation");
  MarkerInterfaceApp interface;
  if (interface.control() < 0)
    ROS_ERROR("End by some fails");
}