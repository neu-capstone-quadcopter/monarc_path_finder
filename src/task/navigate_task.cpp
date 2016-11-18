#include "navigate_task.h"

NavigateTask::NavigateTask(std::shared_ptr<DroneController> drone_controller) 
  : Task(drone_controller) { 
  // Resolve channels.
  std::string octomap_channel = nh_.resolveName("/octomap_binary");

  // Subscribe to all topics.
  octomap_sub_ = nh_.subscribe(octomap_channel, 3, &NavigateTask::octomapCallback, this);
}

void NavigateTask::run() {}

bool NavigateTask::loopOnce() {}

void NavigateTask::octomapCallback(const octomap_msgs::OctomapConstPtr& mapMsg) {
  octomap::AbstractOcTree* abstract_tree = octomap_msgs::binaryMsgToMap(*mapMsg);
  octomap::OcTree* new_map = dynamic_cast<octomap::OcTree*>(abstract_tree);
  if (new_map) {
    {
      std::lock_guard<std::mutex> guard(map_mu_);
      map_.reset(new_map);
    }
  }
}
