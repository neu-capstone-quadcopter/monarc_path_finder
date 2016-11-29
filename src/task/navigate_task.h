#ifndef NAVIGATE_TASK_H_
#define NAVIGATE_TASK_H_

#include <mutex>

#include "octomap/octomap.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"

#include "task.h"

class NavigateTask : public Task {
  // subscriptions
  ros::Subscriber octomap_sub_;

  // octomap members
  std::mutex                       map_mu_;
  std::unique_ptr<octomap::OcTree> map_;

public:
  NavigateTask();

  inline bool  isRunnable(State cur_state) { return cur_state == State::InAir; };
  inline bool  isPreemptible() { return true; };
  State finishState() { return State::InAir; }

  void run();
  bool loopOnce();

private:
  void octomapCallback(const octomap_msgs::OctomapConstPtr& map);
  
};

#endif // NAVIGATE_TASK_H_
