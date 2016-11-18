#ifndef TASK_H_
#define TASK_H_

#include <memory>

#include "ros/ros.h"

#include "drone_controller.h"

enum class State {
  Grounded,
  InAir,
};

class Task {
protected:
  ros::NodeHandle nh_; 

  std::shared_ptr<DroneController> drone_controller_;

public:
  Task(std::shared_ptr<DroneController> drone_controller) :
    drone_controller_(drone_controller) {};
  virtual ~Task() = default;

  virtual bool  isRunnable(State) = 0;
  virtual bool  isPreemptible() = 0;
  virtual State finishState() = 0;

  // run begins the task.
  virtual void run() = 0;
  // loopOnce performs a single task loop, returning if the
  // task has finished executing.
  virtual bool loopOnce() = 0;
};

#endif // TASK_H_
