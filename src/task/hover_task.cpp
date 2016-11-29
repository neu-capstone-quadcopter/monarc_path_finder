#include "hover_task.h"

HoverTask::HoverTask() {

}

void HoverTask::run() {
  waitForActionServer();

  monarc_tf::FlyGoal goal;
  goal.command = monarc_tf::FlyGoal::HOVER;

  ac_.sendGoal(goal, boost::bind(&Task::onActionDone, this, _1, _2));
  running = true;
}

bool HoverTask::loopOnce() {
  return complete;
}
