#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

std::vector<sensor_msgs::JointState> joint_state_msgs;

void callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_state_msgs.push_back(*msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "recorder");

  ros::NodeHandle nh;
  ros::Subscriber recording_sub_ = nh.subscribe<sensor_msgs::JointState>(
      "/unity/recording", 1, callback);

  while (ros::ok())
    ros::spin();
}
