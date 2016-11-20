#include "autopilot.h"

Autopilot::Autopilot()
  : nh_(NODE_NAME) {

  // Private NodeHandle for parameters:
  ros::NodeHandle nh_private("~");
  nh_private.param("loop_frequency", param_loop_frequency_, 10.0);
 
  // Resolve channels.
  std::string task_channel = nh_.resolveName("/tasks");
  std::string gps_channel = nh_.resolveName("/fix");

  // Subscribe to all topics.
  task_sub_ = nh_.subscribe(task_channel, 20, &Autopilot::taskCallback, this);
  gps_sub_ = nh_.subscribe(gps_channel, 5, &Autopilot::gpsCallback, this);
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

void Autopilot::loopOnce() { 
  task_controller_->loop();  
}

void Autopilot::taskCallback(const std_msgs::Int32ConstPtr& task) {
  std::unique_ptr<Task> new_task;
  switch (task->data) {
    case 1:
      new_task = std::make_unique<TakeoffTask>(drone_controller_);
      break;
    case 2:
      new_task = std::make_unique<NavigateTask>(drone_controller_);
      break;
    default:
      ROS_INFO("Unknown message number: %d", task->data);
      return;
  }
  task_controller_->addTask(std::move(new_task));
}

void Autopilot::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& navSatFix) {

}
