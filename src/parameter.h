#ifndef PARAMETER_H
#define PARAMETER_H

#include <ros/ros.h>

extern int MAX_AB;
extern int MAX_POINT;
extern std::vector<std::string> CAM_NAMES;

void readParameters(ros::NodeHandle &n);

#endif