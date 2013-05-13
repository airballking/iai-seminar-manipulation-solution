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

#include <iai_seminar_manipulation_executive/RobotArm.h>
#include <iai_seminar_manipulation_utils/ParameterServerUtils.h>

RobotArm::RobotArm(ros::NodeHandle& n, const std::string& name)
{
  trajectory_client_ = new TrajectoryClient(n, name, true);
  configured_ = false;
}

bool RobotArm::initGoal(const std::vector<std::string>& joint_names, const std::vector<double>& joint_goals,
                        const std::vector<double>& joint_goal_velocities, double duration)
{
  // check input
  if(joint_names.size() == 0)
  {
    ROS_ERROR("[RobotArm::initGoal] Provided vector of joint names was empty!");
    return false;
  }
  if(joint_goals.size() == 0)
  {
    ROS_ERROR("[RobotArm::initGoal] Provided vector of joint goals was empty!");
    return false;
  }
  if(joint_names.size() != joint_goals.size())
  {
    ROS_ERROR("[RobotArm::initGoal] Provided vectors of joint names and goals had different sizes!");
    return false;
  }
  if(joint_goal_velocities.size() != joint_goals.size())
  {
    ROS_ERROR("[RobotArm::initGoal] Provided vectors of joint names and goal velocities had different sizes!");
    return false;
  }
  if(!(duration > 0.0))
  {
    ROS_ERROR("[RobotArm::initGoal] Provided duration was a non-positive number.");
  }

  // set the joint_names
  goal_.trajectory.joint_names = joint_names;

  // set only one trajectory point with goal and velocity
  goal_.trajectory.points.resize(1);
  goal_.trajectory.points[0].positions = joint_goals;
  goal_.trajectory.points[0].velocities = joint_goal_velocities;

  // set duration for only trajectory point
  goal_.trajectory.points[0].time_from_start = ros::Duration(duration); 
 
  // all went fine
  configured_ = true;
  return true;
}

bool RobotArm::initGoal(ros::NodeHandle& n)
{
  // extract parameters from parameter server
  std::vector<std::string> joint_names;
  if(!loadStringVectorFromParameterServer(n, "joints", joint_names))
  {
    ROS_ERROR("[RobotArm::initGoal] Could not load joint names from parameter server.");
    return false;
  }

  std::vector<double> joint_goals;
  if(!loadDoubleVectorFromParameterServer(n, "positions", joint_goals))
  {
    ROS_ERROR("[RobotArm::initGoal] Could not load joint goals from parameter server.");
    return false;
  }

  std::vector<double> goal_velocities;
  if(!loadDoubleVectorFromParameterServer(n, "velocities", goal_velocities))
  {
    ROS_ERROR("[RobotArm::initGoal] Could not load joint goals from parameter server.");
    return false;
  }

  double duration;
  if(!loadDoubleFromParameterServer(n, "execution_time", duration))
  {
    ROS_ERROR("[RobotArm::initGoal] Could not load execution time from parameter server.");
    return false;
  }

  // perform initialization
  return initGoal(joint_names, joint_goals, goal_velocities, duration);
}

bool RobotArm::waitForActionServer()
{
  if(trajectory_client_)
  {
    while(!trajectory_client_->waitForServer(ros::Duration(1.0)))
      ROS_INFO("[RobotArm:waitForActionServer] Waiting for action server to show up.");
    return true;
  }
  else
  {
    // trajectory_client_ was null!
    ROS_ERROR("[RobotArm::waitForActionServer] Trajectory client is NULL!");
    return false;
  }
}

bool RobotArm::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal& goal)
{
  // start motion in 1s from now
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  if(trajectory_client_)
  {
    trajectory_client_->sendGoal(goal);
    return true;
  }
  else
  {
    // trajectory_client_ was null!
    ROS_ERROR("[RobotArm::startTrajectory] Trajectory client is NULL!");
    return false;
  }
}

bool RobotArm::startTrajectory()
{
  if(configured_)
    return startTrajectory(goal_);

  // we were called even though the prior init was successful
  return false;
}
