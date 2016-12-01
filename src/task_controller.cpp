#include "task_controller.h"

void TaskController::addTask(std::unique_ptr<Task> new_task) {
  State prev_state = cur_state_;
  if (!task_queue_.empty()) {
    prev_state = task_queue_.back()->finishState();
  }
  if (new_task->isRunnable(prev_state)) {
    task_queue_.push_back(std::move(new_task));  
  } else {
    ROS_WARN("Task not runnnable in current state");
  }
}

void TaskController::replaceTask(std::unique_ptr<Task> new_task) {
  // Remove all preemptible tasks.
  while (!task_queue_.empty()) {
    if (task_queue_.back()->isPreemptible()) {
      task_queue_.pop_back();
    }
  }
  addTask(std::move(new_task));
}

void TaskController::loop() {
  if (!task_queue_.empty()) {
    // Run the front task if it is not running
    if (!task_queue_.front()->isRunning()) {
      task_queue_.front()->run();
    }

    // Loop and check if the task is finished.
    bool finished = task_queue_.front()->loopOnce();
    if (finished) {
      // Set new state.
      cur_state_ = task_queue_.front()->finishState();

      // Pop the front task.
      task_queue_.pop_front();
    }
  } else {
    switch (cur_state_) {
      case State::Grounded:
        break;
      case State::InAir:
        addTask(std::make_unique<HoverTask>());
        loop();
        break;
      default:
        throw std::invalid_argument("unexpected State value");
    }
  }
}

