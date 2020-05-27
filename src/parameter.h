#ifndef PARAMETER_H
#define PARAMETER_H

#include <ros/ros.h>
#include "camodocal/camera_models/CameraFactory.h"

extern int MAX_AB;
extern std::vector<std::string> CAM_NAMES;

void readParameters(ros::NodeHandle &n);
void readIntrinsicParameter(const std::vector<std::string> &calib_file, std::vector<camodocal::CameraPtr> m_camera);

#endif