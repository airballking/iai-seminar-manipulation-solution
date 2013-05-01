#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>

#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <realtime_tools/realtime_publisher.h>

#include <vector>

class IaiSeminarMultiJointPositionController: public pr2_controller_interface::Controller
{
private:
  std::vector<pr2_mechanism_model::JointState*> joints_;
  realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointTrajectoryControllerState> realtime_publisher_; 

public:
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n);
  virtual void starting();
  virtual void update();
  virtual void stopping();
};
