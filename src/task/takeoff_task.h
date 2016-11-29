#ifndef TAKEOFF_TASK_H_
#define TAKEOFF_TASK_H_

#include "basic_task.h"

class TakeoffTask : public BasicTask {
public:
  inline bool  isRunnable(State cur_state) { return cur_state == State::Grounded; };
  inline bool  isPreemptible() { return false; };
  inline State finishState() { return State::InAir; };
  inline int   goal() { return monarc_tf::FlyGoal::TAKEOFF; };
};

#endif // TAKEOFF_TASK_H_
