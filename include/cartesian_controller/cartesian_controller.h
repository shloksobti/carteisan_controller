#include <iostream>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <mutex>
#include <urdf/model.h>
#include <thread>
//#include <robot_controllers_interface/joint_handle.h>

class CartesianController {
 public:
  CartesianController();
  virtual ~CartesianController(){}
  int init(ros::NodeHandle& nh);
  bool start();
  bool stop();  // Stop controller once path is over.
  void command(const geometry_msgs::Twist::ConstPtr& goal);
  void update(const ros::Time& now, const ros::Duration& dt);
  void updateJointState(const sensor_msgs::JointState::ConstPtr& state);

 private:
  bool initialized_;
  bool is_active_;
  KDL::Chain kdl_chain_;
  boost::shared_ptr<KDL::ChainIkSolverVel_wdls> solver_;
  boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fksolver_;

  KDL::JntArray tgt_jnt_pos_;
  KDL::JntArray tgt_jnt_vel_;
  KDL::JntArray last_tgt_jnt_vel_;

  KDL::Twist twist_command_;
  ros::Subscriber twist_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher joint_state_pub_;

  ros::Time last_command_time_;
  sensor_msgs::JointState joint_state_;
  std::mutex mymutex_;
  std::vector<std::tuple<double, double>> joint_limits_;
  
};
