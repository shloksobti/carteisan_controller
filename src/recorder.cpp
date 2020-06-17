#include <thread>
#include <recorder.h>

Recorder::Recorder(ros::NodeHandle& nh, std::string filename) {
  nh_ = nh;
  joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>(
      "/unity/recording", 1, boost::bind(&Recorder::callback, this, _1));
  file_.open(filename);
  stop_ = false;
}

Recorder::~Recorder() {}

void Recorder::callback(const sensor_msgs::JointState::ConstPtr& msg) {
  ROS_INFO("Message Received");
  sensor_msgs::JointState js;
  for (unsigned i = 0; i <  msg->name.size(); i++)
  {
    js.name.push_back(msg->name[i]);
    js.position.push_back(msg->position[i]);
    js.velocity.push_back(msg->velocity[i]);
    js.effort.push_back(msg->effort[i]);
  }
  joint_states_.push_back(js);
}

bool Recorder::serialize() {
  ROS_INFO("SERIALIZING: %d", (int)joint_states_[0].name.size());
  file_ << joint_states_[0].name.size();
  file_ << ' ';
  file_ << joint_states_.size();
  file_ << ' ';
  for (unsigned d = 0; d < joint_states_[0].name.size(); d++) {
    for (unsigned p = 0; p < joint_states_.size(); p++) {
      file_ << joint_states_[p].position[d];
      file_ << ' ';
    }
  }
  file_.close();
  return true;
}

void Recorder::stopThread()
{
  int n;
  while(1)
  {
    n = std::cin.get();
    if (n == (int)'\n')
    {
      ROS_INFO("Shutting Down");
      break;
    }
  }
  stop_ = true;
  joint_state_sub_.shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "recorder");
  ros::NodeHandle nh;
  std::string filename =
      ros::package::getPath("hybrid_planner") + "/demos/pour.txt";

  Recorder* recorder = new Recorder(nh, filename);
  std::thread stop(&Recorder::stopThread, recorder);

  while (!recorder->stop_ && ros::ok()) 
  {
    ros::spinOnce();
  }

  recorder->serialize();
  ROS_INFO("Done writing to file");

  stop.join();
  delete(recorder);
}
