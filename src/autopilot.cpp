#include "autopilot.h"

Autopilot::Autopilot()
  : nh(NODE_NAME) {


}

void Autopilot::run() {
  try {
    // TODO(nvanbenschoten)

    ros::spin();
  } catch (...) {
    ros::shutdown();
  }
}
