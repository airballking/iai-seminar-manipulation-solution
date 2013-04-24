#include <iai_seminar_manipulation_executive/RobotArm.h>

RobotArm::RobotArm(ros::NodeHandle& n, const std::string& name)
{
  trajectory_client_ = new TrajectoryClient(n, name);
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
  // TODO(Georg): implement this
  return false;
}

bool RobotArm::waitForActionServer(double time_out)
{
  if(trajectory_client_)
  {
    return trajectory_client_->waitForServer(ros::Duration(time_out));
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
