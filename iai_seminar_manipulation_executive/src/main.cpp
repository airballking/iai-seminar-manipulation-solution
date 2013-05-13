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

#include <ros/ros.h>
#include <iai_seminar_manipulation_executive/RobotArm.h>
#include <vector>

bool use_standard_arm_interface(RobotArm& arm)
{
  // construct extra nodehandle with namespace of first goal configuration
  ros::NodeHandle nh("~first_goal_configuration");
  if(!arm.initGoal(nh))
  {
    ROS_ERROR("[iai_seminar_manipulation_executive] Standard arm init failed.");
    return false;
  }

  // wait for server to show
  if(!arm.waitForActionServer())
  {
    ROS_ERROR("[iai_seminar_manipulation_executive] Standard arm action server did not show.");
    return false;
  }

  // start the motion
  if(!arm.startTrajectory())
  {
    ROS_ERROR("[iai_seminar_manipulation_executive] Standard arm starting failed.");
    return false;
  }
 
  // periodically see if the motion successfully finished
  ros::Duration sleep_time = ros::Duration(0.01);
  while(ros::ok() && !arm.getState().isDone())
  {
    sleep_time.sleep();
  }

  // all went fine
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manipulation_executive");
  ros::NodeHandle n("~");

  RobotArm arm(n, "arm_trajectory_action");

  // move arm using standard action interface
  ROS_INFO("[iai_seminar_manipulation_executive] Using standard action interface to move arm...");
  if(!use_standard_arm_interface(arm))
    return 0; 

  // all nice
  return 0;
}
