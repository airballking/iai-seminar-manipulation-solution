#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <vector>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajectoryClient;

class RobotArm
{
public:
  /** Construct the arm interface. Uses the NodeHandle to create a pointer to
      the action-client in a given namespace.

      \param n [in] NodeHandle to the namespace in which its action client
                    shall be created.
      \param name [in] name of the action-interface in the provided namespace.
  */
  RobotArm(ros::NodeHandle& n, const std::string& name);

  // Cleans up the arm interface.
  ~RobotArm()
  {
    if(trajectory_client_)
      delete trajectory_client_;
  }

  /** Initializes the goal of the arm interface to a given goal. Uses the
      explicitedly given parameters to init the goal.

      \param joint_names [in] N joint_names of the joints that shall be moved.
      \param joint_goals [in] N joint_values that shall be reached by the motion.
      \parma joint_goal_velocities [in] N joint_velocities at the end of the motion.
      \param duration [in] amount of time in seconds the motion shall last.
      \return [out] true if successful, otherwise false.
  */
  bool initGoal(const std::vector<std::string>& joint_names, const std::vector<double>& joint_goals,
                const std::vector<double>& joint_goal_velocities, double duration);

  /** Initializes the goal of the arm interface. Reads the desired goal from the
      parameter server.

      \param n [in] namespace in which the goal description can be found.
      \return [out] true if successful, otherwise false.
  */
  bool initGoal(ros::NodeHandle& n);

  /** Waits for the action server to show up.
     
      \return [out] true if server showed, else false.
  */
  bool waitForActionServer();

  /** Starts a trajectory using an externally provided goal.
  
      \param goal [in] goal to send to the action server.
      \return [out] true if the trajectory was started, else false.
  */ 
  bool startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal& goal);

  /** Starts a trajectory using the internal initialized goal. Can only do
      so if the class already been successfully initialized.
  
      \return [out] true if the trajectory was started, else false. 
  */ 
  bool startTrajectory();

  /** Retrieves the current state of the action client.
 
      \returns [out] the current state of the action client. 
  */ 
  actionlib::SimpleClientGoalState getState()
  {
    // Note: This is a hacky way of running into a null-pointer.
    // However, we know at least where it happened.
    assert(trajectory_client_);
    
    return trajectory_client_->getState();
  }
 
private:
  // pointer to our action client to send the requested arm motion out
  TrajectoryClient* trajectory_client_;

  // container for the initialized goal to send to the action server 
  pr2_controllers_msgs::JointTrajectoryGoal goal_;

  // a flag to signal that initialization of the interface was successful
  bool configured_; 
};
