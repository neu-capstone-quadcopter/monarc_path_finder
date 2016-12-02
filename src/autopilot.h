#ifndef AUTOPILOT_H_
#define AUTOPILOT_H_

#include <memory>
#include <string>
#include <utility>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "monarc_uart_driver/NavCommand.h"

#include "octomap/octomap.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"

#include "task_controller.h"
#include "task/task.h"
#include "task/land_task.h"
#include "task/takeoff_task.h"
#include "task/navigate_task.h"
#include "watchdog.h"

#define NODE_NAME "monarc_path_finder_node"

class Autopilot {
  ros::NodeHandle nh_;

  // params
  double param_loop_frequency_;
 
  // subscriptions
  ros::Subscriber command_sub_;
  ros::Subscriber octomap_sub_;

  // task controller
  std::unique_ptr<TaskController> task_controller_;

  // watchdog
  Watchdog watchdog_;
public:
  Autopilot();

  void run();
private:
  void loopOnce();

  void commandCallback(const monarc_uart_driver::NavCommandConstPtr&);
  void octomapCallback(const octomap_msgs::OctomapConstPtr& map) {};
};

#endif // AUTOPILOT_H_
