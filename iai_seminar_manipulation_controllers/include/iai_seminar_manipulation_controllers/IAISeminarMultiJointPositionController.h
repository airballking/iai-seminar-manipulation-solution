#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>

#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <realtime_tools/realtime_publisher.h>

#include <std_msgs/Float64MultiArray.h>

#include <boost/thread/recursive_mutex.hpp>

#include <control_toolbox/pid.h>

#include <vector>

class IaiSeminarMultiJointPositionController: public pr2_controller_interface::Controller
{
private:
  // internal pointers to all the joints to control
  std::vector<pr2_mechanism_model::JointState*> joints_;
  // internal pointer to the robot object
  pr2_mechanism_model::RobotState* robot_;
  // vector holding a pid-controller for every joint to control
  std::vector<control_toolbox::Pid> pids_; 

  // container to store the last time our update was called, needed to calculate time between cycles
  ros::Time last_time_;
  // container to hold the calculated error between actual and desired positions of all joints to control
  std::vector<double> error_;
 
  // a real-time-safe publisher to publish the state of our controller
  realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointTrajectoryControllerState> realtime_publisher_; 

  // double buffers to read command in a realtime-safe fashion
  std::vector<double> position_command_, position_command_buffer_;
  // mutex to guard operations on command buffer
  boost::mutex command_mutex_;

  // subscriber object used to listen to command topic
  ros::Subscriber command_subscriber_;
  // callback to copy in desired joint positions from topic
  void command_callback(const std_msgs::Float64MultiArrayConstPtr& msg);

  // auxiliary function that copies content of position_command_buffer_ into position_command_
  // uses mutex to guard against interference and appear atomic
  void copy_command_buffer();
public:
  // standard interface of pr2 controllers...

  // init gets called once when loading the controller; no need for realtime safety
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n);

  // real-time safe: called once when starting the controller to set it up
  virtual void starting();
 
  // real-time safe: called once every 1ms to computer control signal
  virtual void update();

  // real-time safe: called once when stopping the controller
  virtual void stopping();
};
