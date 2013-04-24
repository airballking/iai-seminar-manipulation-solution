#include <ros/ros.h>
#include <iai_seminar_manipulation_executive/RobotArm.h>
#include <vector>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manipulation_executive");
  ros::NodeHandle n("~");

  RobotArm left_arm(n, "left_arm_trajectory_action");
  std::vector<std::string> joint_names;
  std::vector<double> joint_goals;
  std::vector<double> goal_velocities;
  double duration = 4.0;
  joint_names.clear();
  joint_names.push_back("l_shoulder_pan_joint");
  joint_names.push_back("l_shoulder_lift_joint");
  joint_names.push_back("l_upper_arm_roll_joint");
  joint_names.push_back("l_elbow_flex_joint");
  joint_names.push_back("l_forearm_roll_joint");
  joint_names.push_back("l_wrist_flex_joint");
  joint_names.push_back("l_wrist_roll_joint");

  joint_goals.clear();
  joint_goals.push_back(1.05);
  joint_goals.push_back(0.01);
  joint_goals.push_back(0.61);
  joint_goals.push_back(-0.44);
  joint_goals.push_back(-5.6);
  joint_goals.push_back(-0.86);
  joint_goals.push_back(0.16);
 
  goal_velocities.clear();
  for(unsigned int i=0; i<joint_goals.size(); i++)
  {
    goal_velocities.push_back(0.0);
  }

  if(!left_arm.initGoal(joint_names, joint_goals, goal_velocities, duration))
  {
    ROS_ERROR("[iai_seminar_manipulation_executive] Left arm init failed.");
    return 0;
  }

  if(!left_arm.waitForActionServer(5.0))
  {
    ROS_ERROR("[iai_seminar_manipulation_executive] Left arm action server did not show.");
    return 0;
  }

  if(!left_arm.startTrajectory())
  {
    ROS_ERROR("[iai_seminar_manipulation_executive] Left arm starting failed.");
    return 0;
  }
 
  ros::Duration sleep_time = ros::Duration(0.01);
  while(ros::ok() && !left_arm.getState().isDone())
  {
    sleep_time.sleep();
  } 
  // all nice
  return 0;
}
