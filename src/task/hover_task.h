#ifndef HOVER_TASK_H_
#define HOVER_TASK_H_

#include "basic_task.h"

class HoverTask : public BasicTask {
public:
  inline bool  isRunnable(State cur_state) { return cur_state == State::InAir; };
  inline bool  isPreemptible() { return true; };
  inline State finishState() { return State::InAir; };
  inline int   goal() { return monarc_tf::FlyGoal::HOVER; };
};

#endif // HOVER_TASK_H_
