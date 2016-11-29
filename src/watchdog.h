#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"

using namespace std::chrono;

const system_clock::duration WATCHDOG_TIMEOUT = milliseconds(1500);

class Watchdog {
  // subscriptions
  ros::Subscriber heartbeat_sub_;

  // publications
  ros::Publisher kill_pub_;
 
  // state
  system_clock::time_point       latest_heartbeat_;
  std::mutex                     latest_heartbeat_mu_;
  std::unique_ptr<boost::thread> watchdog_thread_; 
public:
  Watchdog(ros::NodeHandle& nh) {
    // Resolve channels.
    std::string heartbeat_channel = nh.resolveName("/heartbeat");
    std::string kill_channel = nh.resolveName("/kill");

    // Subscribe to heartbeat topic.
    heartbeat_sub_ = nh.subscribe(heartbeat_channel, 1, &Watchdog::heartbeadCallback, this);

    // Register to publish on kill topic.
    kill_pub_ = nh.advertise<std_msgs::Bool>(kill_channel, 1);
  }
  
  ~Watchdog() {
    if (watchdog_thread_) {
      watchdog_thread_->join();
    }
  }

private:
  void heartbeadCallback(std_msgs::Empty empty) {
    // If this is the first heartbeat we see, launch the watchdog thread.
    if (!watchdog_thread_) {
      watchdog_thread_ = std::make_unique<boost::thread>(boost::bind(&Watchdog::watcher, this));
    }

    // Update the latest heartbeat timestamp.
    std::lock_guard<std::mutex> guard(latest_heartbeat_mu_);
    latest_heartbeat_ = system_clock::now();
  }

  void watcher() {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
      // Get the latest time in a thread safe manner.
      system_clock::time_point latest;
      {
        std::lock_guard<std::mutex> guard(latest_heartbeat_mu_);
        latest = latest_heartbeat_;
      }
 
      // Compute time since last heartbeat.
      auto now = system_clock::now();
      system_clock::duration diff = now - latest;
      
      // Check if over timeout.
      if (diff > WATCHDOG_TIMEOUT) {
        std_msgs::Bool kill_msg;
        kill_msg.data = true;
        kill_pub_.publish(kill_msg);
      }

      loop_rate.sleep();
    }
  }
};

#endif // AUTOPILOT_H_
