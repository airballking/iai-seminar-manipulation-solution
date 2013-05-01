#include <iai_seminar_manipulation_controllers/IAISeminarMultiJointPositionController.h>
#include <pluginlib/class_list_macros.h>

bool IaiSeminarMultiJointPositionController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n)
{
  return false;
}

void IaiSeminarMultiJointPositionController::starting()
{

}

void IaiSeminarMultiJointPositionController::update()
{

}

void IaiSeminarMultiJointPositionController::stopping()
{

}

// Register our controller to the pluginlib
PLUGINLIB_DECLARE_CLASS(iai_seminar_manipulation_controllers, IaiSeminarMultiJointPositionController, IaiSeminarMultiJointPositionController, pr2_controller_interface::Controller)
