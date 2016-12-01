#ifndef NAVIGATE_TASK_H_
#define NAVIGATE_TASK_H_

#include "sensor_msgs/NavSatFix.h"

#include "task.h"

class NavigateTask : public Task {
  sensor_msgs::NavSatFix goal_location_;

public:
  NavigateTask(sensor_msgs::NavSatFix goal) : goal_location_(goal) {};

  inline bool  isRunnable(State cur_state) { return cur_state == State::InAir; };
  inline bool  isPreemptible() { return true; };
  State finishState() { return State::InAir; };

  void run() {
    waitForActionServer();

    monarc_tf::FlyGoal task_goal;
    task_goal.command = monarc_tf::FlyGoal::NAVIGATE;
    task_goal.command_location = goal_location_;

    ac_.sendGoal(task_goal, boost::bind(&Task::onActionDone, this, _1, _2));
    running = true;
  };

  bool loopOnce() { return complete; };
};

#endif // NAVIGATE_TASK_H_
