#include "takeoff_task.h"

TakeoffTask::TakeoffTask(std::shared_ptr<DroneController> drone_controller)
  : Task(drone_controller) {
  
}

void TakeoffTask::run() {}

bool TakeoffTask::loopOnce() {}
