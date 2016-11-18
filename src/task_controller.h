#ifndef TASK_CONTROLLER_H_
#define TASK_CONTROLLER_H_

#include <deque>
#include <memory>
#include <string>
#include <stdexcept>
#include <utility>

#include "drone_controller.h"
#include "task/task.h"

class TaskController {
  std::shared_ptr<DroneController> drone_controller_;

  State cur_state_;

  std::deque<std::unique_ptr<Task>> task_queue_;

public:
  TaskController(std::shared_ptr<DroneController> drone_controller) : 
    drone_controller_(drone_controller),
    cur_state_(State::Grounded) {};

  bool hasTask() { return task_queue_.size() > 0; };

  void addTask(std::unique_ptr<Task>);
  void replaceTask(std::unique_ptr<Task>);

  void loop();
private:
};

#endif // TASk_CONTROLLER_H_
