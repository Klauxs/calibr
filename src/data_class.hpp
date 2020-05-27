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
};

Pose inter_pose( const Pose& pose_1, const Pose& pose_2, double time )
{
    Sophus::SE3 tmp_SE3 = pose_1.getSE3();

    double a = ( time - pose_1.time ) / ( pose_2.time - pose_1.time ); 
    tmp_SE3 = tmp_SE3 * Sophus::SE3::exp( a * ( tmp_SE3.inverse() * pose_2.getSE3() ).log() );

    Pose tmp_pose( tmp_SE3.translation(), tmp_SE3.unit_quaternion(), time );

    return tmp_pose; 
}

Pose find_pose( vector<Pose>& pose, double time )
{
    int itime = (int) time;
    Pose bpose, epose;
    vector<Pose> tmp_vec;
    auto it = find_if(pose.begin(), pose.end(), [itime](const Pose &it)
                          {
            return it.int_time == itime;
                          });

    if (it != pose.end()) //找到了
    {
        tmp_vec.assign(it-5, it+25); //里程计频率不能超过25Hz
        sort( tmp_vec.begin(), tmp_vec.end(), [time](const Pose& p1, const Pose& p2){return fabs(p1.time-time) < fabs(p2.time-time);});
        if ( tmp_vec[0].time < tmp_vec[1].time )
        {
            bpose = tmp_vec[0];
            epose = tmp_vec[1];
        }
        else
        {
            bpose = tmp_vec[1];
            epose = tmp_vec[0];
        }

        return inter_pose( bpose, epose, time );
    }

   return bpose; 

}

Pose betw_pose( const Pose& bpose, const Pose& epose )
{
    Sophus::SE3 b_SE3 = bpose.getSE3();
    Sophus::SE3 e_SE3 = epose.getSE3();

    Sophus::SE3 tmp_SE3 = b_SE3.inverse() * e_SE3;

    Pose tmp_pose( tmp_SE3.translation(), tmp_SE3.unit_quaternion(), bpose.time );

    return tmp_pose;
}

#endif