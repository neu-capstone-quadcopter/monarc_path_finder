#include "takeoff_task.h"

TakeoffTask::TakeoffTask() {

}

void TakeoffTask::run() {
  waitForActionServer();

  monarc_tf::FlyGoal goal;
  goal.command = monarc_tf::FlyGoal::TAKEOFF;

  ac_.sendGoal(goal, boost::bind(&Task::onActionDone, this, _1, _2));
  running = true;
}

bool TakeoffTask::loopOnce() {
  return complete;
}
