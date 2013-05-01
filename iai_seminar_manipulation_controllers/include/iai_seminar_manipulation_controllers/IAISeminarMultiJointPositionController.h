#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <vector>

class IaiSeminarMultiJointPositionController: public pr2_controller_interface::Controller
{
private:
  pr2_mechanism_model::JointState* joint_state_;

public:
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n);
  virtual void starting();
  virtual void update();
  virtual void stopping();
};
