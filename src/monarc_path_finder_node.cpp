#include <ros/ros.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "monarc_path_finder_node");

  ros::NodeHandle node;

  ros::spin();
  return 0;
};
