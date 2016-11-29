#ifndef BASIC_TASK_H_
#define BASIC_TASK_H_

#include "task.h"

class BasicTask : public Task {
public:
  virtual int goal() = 0;

  void run() {
    waitForActionServer();

    monarc_tf::FlyGoal task_goal;
    task_goal.command = goal();

    ac_.sendGoal(task_goal, boost::bind(&Task::onActionDone, this, _1, _2));
    running = true;
  }

  bool loopOnce() {
    return complete;
  }
};

#endif // BASIC_TASK_H_
