#ifndef AUTOPILOT_H_
#define AUTOPILOT_H_

#include <memory>
#include <string>
#include <utility>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"
#include "monarc_uart_driver/NavCommand.h"

#include "task_controller.h"
#include "drone_controller.h"
#include "task/task.h"
#include "task/takeoff_task.h"
#include "task/navigate_task.h"

#define NODE_NAME "monarc_path_finder_node"

class Autopilot {
  ros::NodeHandle nh_;

  // params
  double param_loop_frequency_;
 
  // subscriptions
  ros::Subscriber task_sub_;
  ros::Subscriber gps_sub_;

  // task controller
  std::unique_ptr<TaskController> task_controller_;

  // drone controller
  std::shared_ptr<DroneController> drone_controller_;
public:
  Autopilot();

  void run();
private:
  void loopOnce();

  void taskCallback(const std_msgs::Int32ConstPtr& task);  
  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& navSatFix);
};

#endif // AUTOPILOT_H_
