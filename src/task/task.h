#ifndef TASK_H_
#define TASK_H_

#include <memory>
#include <stdexcept>

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

#include "monarc_tf/FlyAction.h"
#include "monarc_tf/FlyGoal.h"

enum class State {
  Grounded,
  InAir,
};

class Task {
protected:
  ros::NodeHandle nh_; 

  bool running;
  bool complete;
  actionlib::SimpleActionClient<monarc_tf::FlyAction> ac_;

  void waitForActionServer();
  bool isActionComplete(actionlib::SimpleClientGoalState);

public:
  Task() :
    ac_("fly", true),
    running(false),
    complete(false) {};
  virtual ~Task() = default;

  virtual bool  isRunnable(State) = 0;
  virtual bool  isPreemptible() = 0;
  virtual State finishState() = 0;

  // run begins the task.
  virtual void run() = 0;
  // loopOnce performs a single task loop, returning if the
  // task has finished executing.
  virtual bool loopOnce() = 0;
  // preempt stops the task.
  void preempt();
  // is the task running.
  bool isRunning() { return running; };

  void onActionDone(const actionlib::SimpleClientGoalState& state,
                    const monarc_tf::FlyResultConstPtr& result) { complete = true; };
};

#endif // TASK_H_
