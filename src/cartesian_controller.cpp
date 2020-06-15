#include <cartesian_controller.h>

int CartesianController::init(ros::NodeHandle& nh) {
  initialized_ = true;
  return 0;
}

bool CartesesianController::stop() { return true; }
