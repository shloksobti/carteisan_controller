#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <fstream>

#include <dmp/GetDMPPlan.h>
#include <dmp/LearnDMPFromDemo.h>
#include <dmp/SetActiveDMP.h>
#include <dmp/DMPData.h>
#include <dmp/DMPPoint.h>
#include <dmp/DMPTraj.h>

class Recorder{
  public:
    Recorder(ros::NodeHandle& nh, std::string filename);
    ~Recorder();
    void callback(const sensor_msgs::JointState::ConstPtr& msg);
    void stopThread();
    bool serialize();
    void toDMP(std::string read_from, std::string save_to);
    bool stop_;
  private:
    double** deserialize(std::string filename, int& rows, int& cols);
    dmp::LearnDMPFromDemoResponse makeLFDRequest(int numDims, int numRows,
                                                 int numCols, double** traj,
                                                 double dt, double K_gain,
                                                 double D_gain, int numBases,
                                                 ros::NodeHandle& n);

    void saveDMP(dmp::LearnDMPFromDemoResponse &dmpResponse, std::string filename);
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_;
    std::vector<sensor_msgs::JointState> joint_states_;
    std::ofstream file_;
};
