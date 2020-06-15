#include <iostream>
#include <string>
#include <vector>

#include <geometry_msgs/TwistStamped.h>

#include <ros/ros.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>

class CartesianController {
 public:
  CartesianController();
  virtual ~CartesianController();
  int init(ros::NodeHandle& nh);
  bool start();
  bool stop();  // Stop controller once path is over.
  void command(const geometry_msgs::TwistStamped::ConstPtr& goal);
  // void cartesianVelocityToJointVelocity();

 private:
  bool initialized_;
  KDL::Chain kdl_chain_;
  boost::shared_ptr<KDL::ChainIKSolverVel_wdls> solver_;
  boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fksolver_;
  KDL::Twist twist_command_;
};
