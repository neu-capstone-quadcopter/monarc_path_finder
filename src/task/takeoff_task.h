#ifndef TAKEOFF_TASK_H_
#define TAKEOFF_TASK_H_

#include "task.h"

class TakeoffTask : public Task {
public:
  TakeoffTask(std::shared_ptr<DroneController> drone_controller);  

  inline bool  isRunnable(State cur_state) { return cur_state == State::Grounded; };
  inline bool  isPreemptible() { return false; };
  inline State finishState() { return State::InAir; };

  void run();
  bool loopOnce();
};

#endif // TAKEOFF_TASK_H_
