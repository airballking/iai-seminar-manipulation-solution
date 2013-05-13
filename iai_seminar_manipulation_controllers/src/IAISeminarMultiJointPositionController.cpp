/*  Copyright (c) 2013, Georg Bartels (georg.bartels@cs.uni-bremen.de)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *   
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the Institute for Artificial Intelligence/Universität Bremen
 *      nor the names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iai_seminar_manipulation_controllers/IAISeminarMultiJointPositionController.h>
#include <pluginlib/class_list_macros.h>
#include <iai_seminar_manipulation_utils/ParameterServerUtils.h>

bool IaiSeminarMultiJointPositionController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n)
{
  // remember robot pointer
  robot_ = robot;
  
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

  // read pid-gains from parameter server
  pids_.resize(dof);
  for(unsigned int i=0; i<dof; i++)
  {
    // constructing the namespace in which the gains of the current joint are stored
    std::string gains_ns = n.getNamespace() + "/gains/" + joint_names[i];
    if(!pids_[i].initParam(gains_ns))
    {
      ROS_ERROR("PID gains for joint '%s' could not be found in namespace '%s'.", joint_names[i].c_str(), gains_ns.c_str());
      return false;
    }
  } 

  // initialize both buffers for position command
  position_command_.resize(dof);
  position_command_buffer_.resize(dof);

  // initialize error data structure
  error_.resize(dof);

  // initialize subscriber
  command_subscriber_ = n.subscribe("command", 1, &IaiSeminarMultiJointPositionController::command_callback, this);

  // initializing publisher
  realtime_publisher_.init(n, "state", 1);

  // resizing state-msg within publisher and intializing it with the correct joint-names
  realtime_publisher_.msg_.joint_names = joint_names;
  realtime_publisher_.msg_.actual.positions.resize(dof);
  realtime_publisher_.msg_.actual.velocities.resize(dof);
  realtime_publisher_.msg_.desired.positions.resize(dof);
  realtime_publisher_.msg_.error.positions.resize(dof);
  
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

  // reseting the pid-controllers
  for (unsigned int i=0; i<pids_.size(); i++)
  {
    pids_[i].reset();
  }

  // remembering starting time
  last_time_ = robot_->getTime();
}

void IaiSeminarMultiJointPositionController::update()
{
  assert(joints_.size() == realtime_publisher_.msg_.actual.positions.size());
  assert(joints_.size() == realtime_publisher_.msg_.actual.velocities.size());
  assert(joints_.size() == realtime_publisher_.msg_.desired.positions.size());
  assert(joints_.size() == position_command_.size());
  assert(joints_.size() == error_.size());
  assert(joints_.size() == pids_.size());

  // remember the current time as last time and calculate the dt between now and last cycle
  ros::Duration dt = robot_->getTime() - last_time_;
  last_time_ = robot_->getTime();
 
  // copy content of command buffer into command data structure
  copy_from_command_buffer();

  // calculate the error vector and control using the pid controllers
  for(unsigned int i=0; i<joints_.size(); i++)
  {
    error_[i] = joints_[i]->position_ - position_command_[i];
    joints_[i]->commanded_effort_ = pids_[i].updatePid(error_[i], dt);
  }
 
  // CONTROL FINISHED 
  // publishing state information
  if(realtime_publisher_.trylock())
  {
    // putting time-stamp
    realtime_publisher_.msg_.header.stamp = robot_->getTime();
        // copying data inside message
    for(unsigned i=0; i<joints_.size(); i++)
    {
      // copying current values
      realtime_publisher_.msg_.actual.positions[i] = joints_[i]->position_;
      realtime_publisher_.msg_.actual.velocities[i] = joints_[i]->velocity_;
      // copying desired values
      realtime_publisher_.msg_.desired.positions[i] = position_command_[i];
      // copying the current error
      realtime_publisher_.msg_.error.positions[i] = error_[i];
    }
    realtime_publisher_.unlockAndPublish();
  }
}

void IaiSeminarMultiJointPositionController::stopping()
{
  //nothing to do
}

void IaiSeminarMultiJointPositionController::command_callback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
  assert(msg->data.size() == position_command_buffer_.size());

  boost::mutex::scoped_lock guard(command_mutex_);
  for(unsigned int i=0; i<position_command_buffer_.size(); i++)
  {
    position_command_buffer_[i] = msg->data[i];
  }
}

void IaiSeminarMultiJointPositionController::copy_from_command_buffer()
{
  assert(position_command_.size() == position_command_buffer_.size());

  // copying the command in from the command-buffer
  boost::mutex::scoped_lock guard(command_mutex_);
  for(unsigned int i=0; i< position_command_.size(); i++)
  {  
    position_command_[i] = position_command_buffer_[i];
  }
}
 
// Register our controller to the pluginlib
PLUGINLIB_DECLARE_CLASS(iai_seminar_manipulation_controllers, IaiSeminarMultiJointPositionController, IaiSeminarMultiJointPositionController, pr2_controller_interface::Controller)
