#include "land_task.h"

LandTask::LandTask() {

}

void LandTask::run() {
  waitForActionServer();

  monarc_tf::FlyGoal goal;
  goal.command = monarc_tf::FlyGoal::LAND;

  ac_.sendGoal(goal, boost::bind(&Task::onActionDone, this, _1, _2));
  running = true;
}

bool LandTask::loopOnce() {
  return complete;
}
