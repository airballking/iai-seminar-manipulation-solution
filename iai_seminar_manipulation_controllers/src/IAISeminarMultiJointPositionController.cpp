#include <iai_seminar_manipulation_controllers/IAISeminarMultiJointPositionController.h>
#include <pluginlib/class_list_macros.h>
#include <iai_seminar_manipulation_executive/ParameterServerUtils.h>

bool IaiSeminarMultiJointPositionController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n)
{
  // look up names of joints to control from parameter server
  std::vector<std::string> joint_names;
  joint_names.clear();
  if(!loadStringVectorFromParameterServer(n, "joints", joint_names))
  {
    ROS_ERROR("Could not load joint names from parameter server.");
    return false;
  }

  // look up and save pointers to the joint-states of those joints
  joints_.clear();
  for(unsigned int i=0; i<joint_names.size(); i++)
  {
    pr2_mechanism_model::JointState* joint_pointer = robot->getJointState(joint_names[i]);
    if(!joint_pointer)
    {
      ROS_ERROR("No joint with name '%s' given in robot model.", joint_names[i].c_str());
      return false;
    }
    joints_.push_back(joint_pointer);
  }

  // check if all joints have been calibrated
  for(unsigned int i=0; i<joints_.size(); i++)
  {
    if(!joints_[i]->calibrated_)
    {
      ROS_ERROR("Joint '%s' has not been calibrated.", joints_[i]->joint_->name.c_str());
      return false;
    }
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
