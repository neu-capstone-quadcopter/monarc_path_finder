#include "autopilot.h"

Autopilot::Autopilot()
  : nh_(NODE_NAME) {

  // Private NodeHandle for parameters:
  ros::NodeHandle nh_private("~");
  nh_private.param("loop_frequency", param_loop_frequency_, 10.0);

  // Resolve channels.
  std::string octomap_channel = nh_.resolveName("/octomap_binary");

  // Subscribe to all topics.
  octomap_sub_ = nh_.subscribe("/octomap_binary", 5, &Autopilot::octomapCallback, this);
}

void Autopilot::run() {
  try {
    ros::Rate loop_rate(param_loop_frequency_);
    while (ros::ok()) {
      ros::spinOnce();
      loopOnce();
      loop_rate.sleep();
    }
  } catch (...) {
    ros::shutdown();
  }
}

void Autopilot::octomapCallback(const octomap_msgs::OctomapConstPtr& mapMsg) {
  octomap::AbstractOcTree* abstract_tree = octomap_msgs::binaryMsgToMap(*mapMsg);
  octomap::OcTree* new_map = dynamic_cast<octomap::OcTree*>(abstract_tree);
  if (new_map) {
    std::lock_guard<std::mutex> guard(map_mu_);
    map_.reset(new_map);
  }
}

void Autopilot::loopOnce() {
  std::lock_guard<std::mutex> guard(map_mu_);
  if (!map_) {
    ROS_WARN("OctoMap not loaded yet");
    return;
  }

  for (auto it = map_->begin_leafs(), end = map_->end_leafs(); it != end; ++it) {
    std::cout << "Node center: " << it.getCoordinate();
    std::cout << " Value: " << it->getValue() << "\n";
  }
}
