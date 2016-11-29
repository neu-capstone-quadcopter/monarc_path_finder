#ifndef HOVER_TASK_H_
#define HOVER_TASK_H_

#include "task.h"

class HoverTask : public Task {
public:
  HoverTask();  

  inline bool  isRunnable(State cur_state) { return cur_state == State::InAir; };
  inline bool  isPreemptible() { return true; };
  inline State finishState() { return State::InAir; };

  void run();
  bool loopOnce();
};

#endif // HOVER_TASK_H_
