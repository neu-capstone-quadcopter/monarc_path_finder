#ifndef LAND_TASK_H_
#define LAND_TASK_H_

#include "basic_task.h"

class LandTask : public BasicTask {
public:
  inline bool  isRunnable(State cur_state) { return cur_state == State::InAir; };
  inline bool  isPreemptible() { return false; };
  inline State finishState() { return State::Grounded; };
  inline int   goal() { return monarc_tf::FlyGoal::LAND; };
};

#endif // LAND_TASK_H_
