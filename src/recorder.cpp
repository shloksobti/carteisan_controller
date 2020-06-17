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

double** Recorder::deserialize(std::string filename, int& rows, int& cols) {
  std::ifstream file(filename);
  file >> rows;
  file >> cols;
  double** arr = new double*[rows];
  for (int i = 0; i < rows; i++) {
    arr[i] = new double[cols];
    for (int j = 0; j < cols; j++) file >> arr[i][j];
  }
  return arr;
}

dmp::LearnDMPFromDemoResponse Recorder::makeLFDRequest(
    int numDims, int numRows, int numCols, double** traj, double dt,
    double K_gain, double D_gain, int numBases, ros::NodeHandle& n) {

  std::cout<<"Making a Learn from Demo Request"<<std::endl;
  dmp::DMPTraj demoTraj;
  for (int i = 0; i < numCols; i++) {
    dmp::DMPPoint pt;
    std::vector<double>* positions = new std::vector<double>(numRows);
    ROS_INFO("Checkpoint 1");

    for (int j = 0; j < numRows; j++) {
      positions->at(j) = (traj[j][i]);
    }
    ROS_INFO("Check Point 2");

    for (std::vector<double>::const_iterator i = positions->begin();
         i != positions->end(); ++i)
      std::cout << *i << ' ';
    std::cout << "" << std::endl;

    pt.positions = *positions;
    demoTraj.points.push_back(pt);
    demoTraj.times.push_back(dt * i);
  }

  std::vector<double> kGains, dGains;
  for (int i = 0; i < numRows; i++) {
    kGains.push_back(K_gain);
    dGains.push_back(D_gain);
  }

  std::cout << "Starting LFD..." << std::endl;
  dmp::LearnDMPFromDemoResponse resp;

  // Request Service
  ros::ServiceClient client =
      n.serviceClient<dmp::LearnDMPFromDemo>("learn_dmp_from_demo");
  dmp::LearnDMPFromDemo srv;
  srv.request.demo = demoTraj;
  srv.request.k_gains = kGains;
  srv.request.d_gains = dGains;
  srv.request.num_bases = numBases;
  if (client.call(srv)) {
    resp = srv.response;
    return resp;
  } else {
    ROS_INFO("Failed to call service learn_from_demo");
  }

  return resp;
}

void Recorder::saveDMP(dmp::LearnDMPFromDemoResponse& dmpResponse,
                       std::string filename) {
  double tau = dmpResponse.tau;
  std::ofstream out(filename);
  out << dmpResponse.dmp_list.size() << "\n";  // Number of dimensions
  out << dmpResponse.tau << "\n";
  for (int i = 0; i < dmpResponse.dmp_list.size(); i++) {
    out << dmpResponse.dmp_list[i].d_gain << " ";
    out << dmpResponse.dmp_list[i].k_gain << " \n";
    out << dmpResponse.dmp_list[i].f_domain.size() << "\n";

    for (int j = 0; j < dmpResponse.dmp_list[i].f_domain.size(); j++) {
      out << dmpResponse.dmp_list[i].f_domain[j] << " ";
      out << dmpResponse.dmp_list[i].f_targets[j] << " \n";
    }

    out << dmpResponse.dmp_list[i].weights.size() << "\n";
    for (int k = 0; k < dmpResponse.dmp_list[i].weights.size(); k++) {
      out << dmpResponse.dmp_list[i].weights[k] << "\n";
    }
  }
  out.close();
}

void Recorder::toDMP(std::string read_from, std::string save_to)
{
  int rows, cols;
  double** traj = deserialize(read_from, rows, cols);
  ROS_INFO("Check arr: %f", traj[0][0]);
  ROS_INFO("Rows: %d, Cols: %d", rows, cols);
  auto learnt_dmp = makeLFDRequest(8, rows, cols, traj, 1.0, 100, 2.0*sqrt(100), 4, nh_);
  saveDMP(learnt_dmp, save_to);
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
  std::string save_to = ros::package::getPath("hybrid_planner") + "/DMPs/pour.txt";
  std::string testname = ros::package::getPath("hybrid_planner") + "/demos/place.txt";
  recorder->toDMP(filename, save_to);

  ROS_INFO("Done writing to file");

  stop.join();
  delete(recorder);
}
