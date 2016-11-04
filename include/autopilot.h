#ifndef AUTOPILOT_H_
#define AUTOPILOT_H_

#include <memory>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#define NODE_NAME "monarc_path_finder_node"

class Autopilot {
  ros::NodeHandle nh_;

  // params
  double param_loop_frequency_;

  // subscriptions
  ros::Subscriber octomap_sub_;

  // octomap members
  std::mutex                       map_mu_;
  std::unique_ptr<octomap::OcTree> map_;
public:
  Autopilot();

  void run();
private:
  void octomapCallback(const octomap_msgs::OctomapConstPtr& map);

  void loopOnce();
};

#endif // AUTOPILOT_H_
