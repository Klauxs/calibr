#include "parameter.h"

int MAX_AB;
int MAX_POINT;
std::vector<std::string> CAM_NAMES;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}


void readParameters(ros::NodeHandle &n)
{
    std::string cam_path;
    cam_path = readParam<std::string>(n, "cam0_calib");
    CAM_NAMES.push_back(cam_path);
    cam_path = readParam<std::string>(n, "cam1_calib");
    CAM_NAMES.push_back(cam_path);

    MAX_AB = readParam<int>(n, "MAX_AB");
    MAX_POINT = readParam<int>(n, "MAX_POINT");
}