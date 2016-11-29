#ifndef LAND_TASK_H_
#define LAND_TASK_H_

#include "task.h"

class LandTask : public Task {
public:
  LandTask();  

  inline bool  isRunnable(State cur_state) { return cur_state == State::InAir; };
  inline bool  isPreemptible() { return false; };
  inline State finishState() { return State::Grounded; };

  void run();
  bool loopOnce();
};

#endif // LAND_TASK_H_
