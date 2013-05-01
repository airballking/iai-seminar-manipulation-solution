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
  unsigned int dof = joint_names.size();

  // look up and save pointers to the joint-states of those joints
  joints_.clear();
  for(unsigned int i=0; i<dof; i++)
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
  for(unsigned int i=0; i<dof; i++)
  {
    if(!joints_[i]->calibrated_)
    {
      ROS_ERROR("Joint '%s' has not been calibrated.", joints_[i]->joint_->name.c_str());
      return false;
    }
  }

  // initialize both buffers for position command
  position_command_.resize(dof);
  position_command_buffer_.resize(dof);

  // initializing publisher
  realtime_publisher_.init(n, "state", 1);

  // resizing state-msg within publisher and intializing it with the correct joint-names
  realtime_publisher_.msg_.joint_names = joint_names;
  realtime_publisher_.msg_.actual.positions.resize(dof);
  realtime_publisher_.msg_.actual.velocities.resize(dof);
  realtime_publisher_.msg_.desired.positions.resize(dof);
  realtime_publisher_.msg_.desired.velocities.resize(dof);
  realtime_publisher_.msg_.error.positions.resize(dof);
  realtime_publisher_.msg_.error.velocities.resize(dof);
  
  return true;
}

void IaiSeminarMultiJointPositionController::starting()
{
  // set both command buffers to current configuration
  boost::mutex::scoped_lock guard(command_mutex_);
  for(unsigned int i=0; i<joints_.size(); i++)
  {
    position_command_[i] = joints_[i]->position_;
    position_command_buffer_[i] = joints_[i]->position_; 
  }
  guard.unlock();
}

void IaiSeminarMultiJointPositionController::update()
{
  // publishing state information
  if(realtime_publisher_.trylock())
  {
    // putting time-stamp
    realtime_publisher_.msg_.header.stamp = ros::Time::now();
    // copying current values
    if(joints_.size() == realtime_publisher_.msg_.actual.positions.size())
    {
      for(unsigned i=0; i<joints_.size(); i++)
      {
        realtime_publisher_.msg_.actual.positions[i] = joints_[i]->position_;
      }
    }
    else
    {
      ROS_ERROR("Size of message and internal data structures of actual joint positions do not match.");
    }
    if(joints_.size() == realtime_publisher_.msg_.actual.velocities.size())
    {
      for(unsigned i=0; i<joints_.size(); i++)
      {
        realtime_publisher_.msg_.actual.velocities[i] = joints_[i]->velocity_;
      }
    }
    else
    {
      ROS_ERROR("Size of message and internal data structures of actual joint velocities do not match.");
    }
    realtime_publisher_.unlockAndPublish();
  }
}

void IaiSeminarMultiJointPositionController::stopping()
{

}

// Register our controller to the pluginlib
PLUGINLIB_DECLARE_CLASS(iai_seminar_manipulation_controllers, IaiSeminarMultiJointPositionController, IaiSeminarMultiJointPositionController, pr2_controller_interface::Controller)
