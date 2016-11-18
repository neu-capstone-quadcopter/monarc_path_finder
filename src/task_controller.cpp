#include "task_controller.h"

void TaskController::addTask(std::unique_ptr<Task> new_task) {
  State prev_state = cur_state_;
  if (!task_queue_.empty()) {
    prev_state = task_queue_.back()->finishState();
  }
  if (new_task->isRunnable(prev_state)) {
    task_queue_.push_back(std::move(new_task));  
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
    bool finished = task_queue_.front()->loopOnce();
    if (finished) {
      task_queue_.pop_front();
    }
  } else {
    switch (cur_state_) {
      case State::Grounded:
        break;
      case State::InAir:
	drone_controller_->hover();
        break;
      default:
        throw std::invalid_argument("unexpected State value");
    }
  }
}
