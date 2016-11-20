#include "autopilot.h"

Autopilot::Autopilot()
  : nh_(NODE_NAME) {

  // Private NodeHandle for parameters:
  ros::NodeHandle nh_private("~");
  nh_private.param("loop_frequency", param_loop_frequency_, 10.0);
 
  // Resolve channels.
  std::string command_channel = nh_.resolveName("/nav_command");
  std::string gps_channel = nh_.resolveName("/fix");

  // Subscribe to all topics.
  command_sub_ = nh_.subscribe(command_channel, 20, &Autopilot::commandCallback, this);
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

void Autopilot::commandCallback(const monarc_uart_driver::NavCommandConstPtr& command) {
  std::unique_ptr<Task> new_task;
  switch (command->command_number) {
    case monarc_uart_driver::NavCommand::TAKEOFF:
      new_task = std::make_unique<TakeoffTask>(drone_controller_);
      break;
    case monarc_uart_driver::NavCommand::NAVIGATE_TO_GOAL:
      new_task = std::make_unique<NavigateTask>(drone_controller_);
      break;
    default:
      ROS_INFO("Unhandled command type: %d", command->command_number);
      return;
  }
  task_controller_->addTask(std::move(new_task));
}

void Autopilot::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& location) {
  task_controller_->setLocation(location);
}
