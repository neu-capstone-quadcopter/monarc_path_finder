#include "autopilot.h"

Autopilot::Autopilot()
  : nh_(NODE_NAME),
    task_controller_(std::make_unique<TaskController>()),
    watchdog_(nh_) {

  // Private NodeHandle for parameters:
  ros::NodeHandle nh_private("~");
  nh_private.param("loop_frequency", param_loop_frequency_, 10.0);
 
  // Resolve channels.
  std::string command_channel = nh_.resolveName("/nav_command");

  // Subscribe to all topics.
  command_sub_ = nh_.subscribe(command_channel, 20, &Autopilot::commandCallback, this);

  // Resolve channels.
  std::string octomap_channel = nh_.resolveName("/octomap_binary");

  // Subscribe to all topics.
  octomap_sub_ = nh_.subscribe(octomap_channel, 3, &Autopilot::octomapCallback, this);
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
      new_task = std::make_unique<TakeoffTask>();
      break;
    case monarc_uart_driver::NavCommand::LAND:
      new_task = std::make_unique<LandTask>();
      break;
    case monarc_uart_driver::NavCommand::NAVIGATE_TO_GOAL:
      new_task = std::make_unique<NavigateTask>(command->command_location);
      break;
    default:
      ROS_INFO("Unhandled command type: %d", command->command_number);
      return;
  }
  task_controller_->replaceTask(std::move(new_task));
}

