#include <iai_seminar_manipulation_controllers/IAISeminarMultiJointPositionController.h>
#include <pluginlib/class_list_macros.h>
#include <iai_seminar_manipulation_executive/ParameterServerUtils.h>

bool IaiSeminarMultiJointPositionController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n)
{
  std::vector<std::string> joint_names;
  joint_names.clear();
  if(!loadStringVectorFromParameterServer(n, "joints", joint_names))
  {
    ROS_ERROR("Could not load joint names from parameter server.");
    return false;
  }

  return true;
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
