#include "data_class.h"

Pose inter_pose( const Pose& pose_1, const Pose& pose_2, double time )
{
    Sophus::SE3 tmp_SE3 = pose_1.getSE3();

    double a = ( time - pose_1.time ) / ( pose_2.time - pose_1.time ); 
    tmp_SE3 = tmp_SE3 * Sophus::SE3::exp( a * ( tmp_SE3.inverse() * pose_2.getSE3() ).log() );

    Pose tmp_pose( tmp_SE3.translation(), tmp_SE3.unit_quaternion(), time );

    return tmp_pose; 
}

Pose betw_pose( const Pose& bpose, const Pose& epose )
{
    Sophus::SE3 b_SE3 = bpose.getSE3();
    Sophus::SE3 e_SE3 = epose.getSE3();

    Sophus::SE3 tmp_SE3 = b_SE3.inverse() * e_SE3;

    Pose tmp_pose( tmp_SE3.translation(), tmp_SE3.unit_quaternion(), epose.time - bpose.time );

    return tmp_pose;
}

vector<Pose> find_pose( const vector<Pose>& pose, double time )
{
    int itime = (int) time;
    Pose bpose, epose;
    vector<Pose> tmp_vec, re_pose;
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

        re_pose.push_back(inter_pose( bpose, epose, time ));
        re_pose.push_back(betw_pose(bpose, epose));
        return re_pose;
    }

    re_pose.push_back( bpose );
    return re_pose; 

}