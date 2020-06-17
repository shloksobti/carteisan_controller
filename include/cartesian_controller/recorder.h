#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <fstream>

class Recorder{
  public:
    Recorder(ros::NodeHandle& nh, std::string filename);
    ~Recorder();
    void callback(const sensor_msgs::JointState::ConstPtr& msg);
    void stopThread();
    bool serialize();
    bool stop_;
  private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_;
    std::vector<sensor_msgs::JointState> joint_states_;
    std::ofstream file_;
};
