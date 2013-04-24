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
