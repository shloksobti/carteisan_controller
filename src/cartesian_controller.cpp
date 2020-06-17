#include <cartesian_controller.h>

CartesianController::CartesianController() {
  initialized_ = false;
  is_active_ = false;

  //joint_limits_.emplace_back(0, 0.38615); // Torso Lift Joint
  joint_limits_.emplace_back(-1.6056, 1.6056); // Shoulder Pan Joint
  joint_limits_.emplace_back(-1.221, 1.518); // Shoulder Lift Joint
  joint_limits_.emplace_back(-10000, 10000); // Upperarm Roll Joint
  joint_limits_.emplace_back(-2.251, 2.251); // Elbow Flex Joint
  joint_limits_.emplace_back(-10000, 10000); // Forearm Roll Joint
  joint_limits_.emplace_back(-2.16, 2.16); // Wrist Flex Joint
  joint_limits_.emplace_back(-10000, 10000); // Wrist Roll Joint
}

int CartesianController::init(ros::NodeHandle& nh) {
  // Initialize KDL Structures
  std::string tip_link, root_link;
  nh.param<std::string>("root_name", root_link, "torso_lift_link");
  nh.param<std::string>("tip_name", tip_link, "wrist_roll_link");

  // Load URDF
  urdf::Model model;
  std::string urdf_file =
      ros::package::getPath("fetch_description") + "/robots/fetch.urdf";
  if (!model.initFile(urdf_file)) {
    ROS_ERROR("Failed to parse URDF");
    return -1;
  }
  ROS_INFO("Succesfully parsed URDF file");

  // Load the tree
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
    ROS_ERROR("Could not construct tree from URDF");
    return -1;
  }

  // Populate the Chain
  if (!kdl_tree.getChain(root_link, tip_link, kdl_chain_)) {
    ROS_ERROR("Could not construct chain from URDF");
    return -1;
  }

  solver_.reset(
      new KDL::ChainIkSolverVel_wdls(kdl_chain_));  // Main IK Velocity Solver
  fksolver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  unsigned num_joints = kdl_chain_.getNrOfJoints();

  tgt_jnt_pos_.resize(num_joints);  // Target Joint Positions
  tgt_jnt_vel_.resize(
      num_joints);  // Velocities gotten by IK that joints need to have
  last_tgt_jnt_vel_.resize(num_joints);  // Last Joint Velocities

  for (unsigned ii = 0; ii < num_joints; ++ii) {
    last_tgt_jnt_vel_(ii) = 0.0;
  }

  // Subscribe to Twist Value and assign Command() as callback.
  twist_sub_ = nh.subscribe<geometry_msgs::Twist>(
      "/unity/twist", 1, boost::bind(&CartesianController::command, this, _1));

  joint_state_pub_ =
      nh.advertise<sensor_msgs::JointState>("/ros/joint_state", 1);

  ROS_INFO("Printing KDL Info");
  for (auto& segment : kdl_chain_.segments) {
    std::string name = segment.getName();
    ROS_INFO("Segment Name: %s", name.c_str());
  }
  
  // Instead of this get from Unity.
  //joint_state_.name.push_back("torso_lift_joint");
  joint_state_.name.push_back("shoulder_pan_joint");
  joint_state_.name.push_back("shoulder_lift_joint");
  joint_state_.name.push_back("upperarm_roll_joint");
  joint_state_.name.push_back("elbow_flex_joint");
  joint_state_.name.push_back("forearm_roll_joint");
  joint_state_.name.push_back("wrist_flex_joint");
  joint_state_.name.push_back("wrist_roll_joint");
  //joint_state_.position.push_back(0);
  joint_state_.position.push_back(-2.980231954552437e-07);
  joint_state_.position.push_back(0);
  joint_state_.position.push_back(0);
  joint_state_.position.push_back(-1.7881393432617188e-07);
  joint_state_.position.push_back(0);
  joint_state_.position.push_back(0);
  joint_state_.position.push_back(-1.1920928244535389e-07);
  for (unsigned int i = 0; i < joint_state_.name.size(); i++) joint_state_.velocity.push_back(0);

  initialized_ = true;
  ROS_INFO("Cartesian Controller Initialized");
  return 0;
}

bool CartesianController::start() {
  ROS_INFO("Start Called");
  if (!initialized_) {
    ROS_ERROR_NAMED("CartesianTwistController",
                    "Unable to start, not initialized.");
    is_active_ = false;
    return false;
  }

  // Joint State to Joint Array
  for (unsigned ii = 0; ii < joint_state_.position.size(); ++ii) {
    last_tgt_jnt_vel_(ii) = joint_state_.velocity[ii];
    tgt_jnt_pos_(ii) = joint_state_.position[ii];
    ROS_INFO("Start- Tgt Jnt Pos: %f", tgt_jnt_pos_(ii));
  }
  is_active_ = true;
  return true;
}

bool CartesianController::stop() {
  is_active_ = false;
  return true;
};

void CartesianController::command(const geometry_msgs::Twist::ConstPtr& goal) {
  ROS_INFO("Received Twist Command");
  // Need to initialize KDL structs
  if (!initialized_) {
    ROS_ERROR(
        "CartesianTwistController: Cannot accept goal, controller is not "
        "initialized.");
    return;
  }

  KDL::Twist twist;
  twist(0) = goal->linear.x;
  twist(1) = goal->linear.y;
  twist(2) = goal->linear.z;
  twist(3) = goal->angular.x;
  twist(4) = goal->angular.y;
  twist(5) = goal->angular.z;

  ros::Time now(ros::Time::now());
  twist_command_ = twist;
  last_command_time_ = now;

  if (!is_active_) start();

  update(now, ros::Duration(0.01));
}

void CartesianController::update(const ros::Time& now,
                                 const ros::Duration& dt) {
  if (is_active_ && initialized_) {
    KDL::Frame cart_pose;
    KDL::Twist twist;
    ros::Time last_command_time;

    twist = twist_command_;
    last_command_time = last_command_time_;
    unsigned num_joints = joint_state_.position.size();

    if ((now - last_command_time) > ros::Duration(0.5)) {
      ROS_INFO("STOPPING");
      stop();
    }

    // change the twist here
    if (solver_->CartToJnt(tgt_jnt_pos_, twist, tgt_jnt_vel_) < 0) {
      ROS_INFO("Solver not good");
      for (unsigned ii = 0; ii < num_joints; ++ii) {
        tgt_jnt_vel_(ii) = 0.0;
      }
    }

    //double max_vel = 0.0;
    //for (unsigned int ii = 0; ii < num_joints; ++ii)
      //max_vel = std::max(std::abs(tgt_jnt_vel_(ii)), max_vel);

    //ROS_INFO("Max Velocity: %f", max_vel);
    //double joint_velocity_limit = 0.5;
    //double scale = 1.0;
    //if (max_vel > joint_velocity_limit) {
      //double scale = joint_velocity_limit / max_vel;
      //for (unsigned ii = 0; ii < num_joints; ++ii) {
        //tgt_jnt_vel_(ii) *= scale;
      //}
      //ROS_DEBUG_THROTTLE(1.0, "Joint velocity limit reached.");
    //}

    //// Make sure solver didn't generate any NaNs.
    //for (unsigned ii = 0; ii < num_joints; ++ii) {
      //if (!std::isfinite(tgt_jnt_vel_(ii))) {
        //ROS_ERROR_THROTTLE(1.0, "Target joint velocity (%d) is not finite : %f",
                           //ii, tgt_jnt_vel_(ii));
        //tgt_jnt_vel_(ii) = 1.0;
      //}
    //}

    //// Limit accelerations while trying to keep same resulting direction
    //// somewhere between previous and current value
    //scale = 1.0;
    //double accel_limit = 1.0;
    //double vel_delta_limit = accel_limit * dt.toSec();
    //for (unsigned ii = 0; ii < num_joints; ++ii) {
      //double vel_delta = std::abs(tgt_jnt_vel_(ii) - last_tgt_jnt_vel_(ii));
      //if (vel_delta > vel_delta_limit) {
        //scale = std::min(scale, vel_delta_limit / vel_delta);
      //}
    //}

    //if (scale <= 0.0) {
      //ROS_ERROR_THROTTLE(1.0,
                         //"CartesianTwistController: acceleration limit "
                         //"produces non-positive scale %f",
                         //scale);
      //scale = 0.0;
    //}
    //double scale = 1.0;
    // Linear interpolate betwen previous velocity and new target velocity using
    // scale. scale = 1.0  final velocity = new target velocity scale = 0.0
    // final velocity = previous velocity
    double scale = 0.6;
    for (unsigned ii = 0; ii < num_joints; ++ii) {
      tgt_jnt_vel_(ii) = (tgt_jnt_vel_(ii) - last_tgt_jnt_vel_(ii)) * scale +
                         last_tgt_jnt_vel_(ii);
    }

    // Calculate new target position of joint.  Put target position a few
    // timesteps into the future
    double dt_sec = dt.toSec();
    for (unsigned ii = 0; ii < num_joints; ++ii) {
      tgt_jnt_pos_(ii) += tgt_jnt_vel_(ii) * dt_sec;
    }

    // Enforce Joint Limits.
    for (unsigned ii = 0; ii < num_joints; ++ii) {
      if (tgt_jnt_pos_(ii) > std::get<1>(joint_limits_[ii]))
        tgt_jnt_pos_(ii) = std::get<1>(joint_limits_[ii]);
      else if (tgt_jnt_pos_(ii) < std::get<0>(joint_limits_[ii]))
        tgt_jnt_pos_(ii) = std::get<0>(joint_limits_[ii]);
    }

    for (size_t ii = 0; ii < joint_state_.position.size(); ++ii) {
      joint_state_.position[ii] = tgt_jnt_pos_(ii);
      joint_state_.velocity[ii] = last_tgt_jnt_vel_(ii);
      last_tgt_jnt_vel_(ii) = tgt_jnt_vel_(ii);
      ROS_INFO("Target Joint Position: %f", tgt_jnt_pos_(ii));
    }
    // Publish Joint States to Unity
    joint_state_pub_.publish(joint_state_);
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "cartesian_controller");
  ros::NodeHandle nh;
  CartesianController cartesian_controller;
  cartesian_controller.init(nh);
  ros::spin();
}
