#ifndef DATA_CLASS_H
#define DATA_CLASS_H

#include <list>
#include <vector>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <sophus/se3.h>

class FeaturePerFrame
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FeaturePerFrame( Vector2d _uv, double _time)
    {
        uv = _uv;
        time = _time;
    }
    double time;
    Vector2d uv;
};

class FeaturePerId
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    const int feature_id;
    vector<FeaturePerFrame> feature_per_frame;
    Vector3d gt_p;
    int p_flag;

    void add_gt( Vector3d _gt_p )
    {
        gt_p = _gt_p;
        p_flag = 1;
    }

    FeaturePerId(int _feature_id)  
        : feature_id(_feature_id), p_flag(0)
    {
    }

};

class Pose
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double time;
        int int_time;
        Vector3d position;
        Quaterniond quat;

    Pose(Vector3d _position, Quaterniond _quat, double _time)
        : position(_position), quat(_quat), time(_time)
        {
            int_time = (int) time;
        }

    Pose(Vector3d _position, Matrix3d _rotation, double _time)
        : position(_position), time(_time)
        {
            quat = _rotation;
            int_time = (int) time;
        }

    Pose(Vector3d _position, Matrix3d _rotation)
        : position(_position)
        {
            quat = _rotation;
            time = -1;
        }

    Pose()
    {
        time = -1;
    }

    bool operator < (const Pose& p ) const
    {
        return time < p.time;
    }

    Sophus::SE3 getSE3() const
    {
        Sophus::SE3 SE3_qt( quat, position);
        return SE3_qt;
    }

    Sophus::SO3 getSO3() const
    {
        Sophus::SO3 SO3_q( quat );
        return SO3_q;
    }

    Pose inverse()
    {
        Sophus::SE3 tmp_SE3;
        tmp_SE3 = getSE3().inverse();
        Pose tmp_pose( tmp_SE3.translation(), tmp_SE3.unit_quaternion(), time );
        return tmp_pose;
    }
};

Pose inter_pose( const Pose& pose_1, const Pose& pose_2, double time );
Pose betw_pose( const Pose& bpose, const Pose& epose );
vector<Pose> find_pose( const vector<Pose>& pose, double time );

#endif