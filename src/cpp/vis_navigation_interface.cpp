#include <ros/ros.h>

#include <iostream>

#include "navigation_interface/localization_interface.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "vis_navigation_interface");
  ros::NodeHandle nh, nhp("~");

  VISLocalizationInterface loc_interface(nh, nhp);

  ros::spin();
  return 0;
}
