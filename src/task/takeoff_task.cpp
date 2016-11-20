#include "takeoff_task.h"

TakeoffTask::TakeoffTask(std::shared_ptr<DroneController> drone_controller)
  : Task(drone_controller) {

}

void TakeoffTask::run() {
  waitForActionServer();

  monarc_tf::FlyGoal goal;
  goal.command = monarc_tf::FlyGoal::TAKEOFF;

  ac_.sendGoal(goal);
}

bool TakeoffTask::loopOnce() {
  return isActionComplete(ac_.getState());
}
