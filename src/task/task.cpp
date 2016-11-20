#include "task.h"

void Task::waitForActionServer() {
  ROS_INFO("Waiting for action server to start.");
  ac_.waitForServer();
  ROS_INFO("Action service connected.");
}

bool Task::isActionComplete(actionlib::SimpleClientGoalState state) {
  using actionlib::SimpleClientGoalState;
  switch (state.state_) {
    case SimpleClientGoalState::StateEnum::PENDING:
    case SimpleClientGoalState::StateEnum::ACTIVE:
      return false;
    case SimpleClientGoalState::StateEnum::RECALLED:
    case SimpleClientGoalState::StateEnum::REJECTED:
    case SimpleClientGoalState::StateEnum::PREEMPTED:
    case SimpleClientGoalState::StateEnum::ABORTED:
      // TODO handle these states
      throw std::invalid_argument("unhandled SimpleClientGoalState failure state");
    case SimpleClientGoalState::StateEnum::SUCCEEDED:
      return true;
    case SimpleClientGoalState::StateEnum::LOST:
      throw std::invalid_argument("action lost... wtf... get it together ROS");
    default:
      throw std::invalid_argument("unexpected SimpleClientGoalState::StateEnum value");
  }
}
