#include "parameter.h"

int MAX_AB;
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

void readIntrinsicParameter(const std::vector<std::string> &calib_file, std::vector<camodocal::CameraPtr> m_camera)
{
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera);
    }
}

void readParameters(ros::NodeHandle &n)
{
    std::string cam_path;
    cam_path = readParam<std::string>(n, "cam0_calib");
    CAM_NAMES.push_back(cam_path);
    cam_path = readParam<std::string>(n, "cam1_calib");
    CAM_NAMES.push_back(cam_path);

    MAX_AB = readParam<int>(n, "MAX_AB");
}