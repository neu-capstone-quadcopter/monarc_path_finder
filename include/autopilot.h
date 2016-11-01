#ifndef AUTOPILOT_H_
#define AUTOPILOT_H_

#include <ros/ros.h>

#define NODE_NAME "monarc_path_finder_node"

class Autopilot {
  ros::NodeHandle nh;

  ros::Subscriber octomap_sub;
public:
  Autopilot();

  void run();
};

#endif // AUTOPILOT_H_
