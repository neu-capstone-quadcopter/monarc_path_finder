#ifndef TASK_CONTROLLER_H_
#define TASK_CONTROLLER_H_

#include <deque>
#include <memory>
#include <string>
#include <stdexcept>
#include <utility>

#include "task/task.h"
#include "task/hover_task.h"

class TaskController {
  State cur_state_;

  std::deque<std::unique_ptr<Task>> task_queue_;

public:
  TaskController() :
    cur_state_(State::Grounded) {};

  bool hasTask() { return task_queue_.size() > 0; };

  void addTask(std::unique_ptr<Task>);
  void replaceTask(std::unique_ptr<Task>);

  void loop();
private:
};

#endif // TASk_CONTROLLER_H_
