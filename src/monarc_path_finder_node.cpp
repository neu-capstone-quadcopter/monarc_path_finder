#include "ros/ros.h"

#include "autopilot.h"

int main(int argc, char** argv){
  ros::init(argc, argv, NODE_NAME);
  Autopilot autopilot;
  autopilot.run();
  return 0;
};
