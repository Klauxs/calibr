#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Eigenvalues>
#include <map>
#include "parameter.h"
#include "data_class.h"
#include "handeye.hpp"
#include "project_factor.h"
#include "pose_parameterization.h"

vector<FeaturePerId> feature;

int rec_count, bflag = 0;
using namespace ceres;

void vodom_callback(const nav_msgs::OdometryPtr &vodom_msg)
{
    double tmp_t;
    Vector3d tmp_p;
    Quaterniond tmp_q;
    tmp_t = vodom_msg->header.stamp.toSec();
    tmp_p.x() = vodom_msg->pose.pose.position.x;
    tmp_p.y() = vodom_msg->pose.pose.position.y;
    tmp_p.z() = vodom_msg->pose.pose.position.z;
    tmp_q.w() = vodom_msg->pose.pose.orientation.w;
    tmp_q.x() = vodom_msg->pose.pose.orientation.x;
    tmp_q.y() = vodom_msg->pose.pose.orientation.y;
    tmp_q.z() = vodom_msg->pose.pose.orientation.z;

    // cout << " receive vodom " << endl;
    // cout << vpose.size() << endl;

    vpose.push_back( Pose(tmp_p, tmp_q, tmp_t) );
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    double tmp_t;
    int tmp_id;
    Vector2d tmp_uv;

    if ( bflag == 0 )
        bflag = 1;
    
    rec_count = 0;

    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        tmp_t = feature_msg->header.stamp.toSec();
        tmp_uv(0) = feature_msg->points[i].x;
        tmp_uv(1) = feature_msg->points[i].y;
        tmp_id = feature_msg->points[i].z;

        FeaturePerFrame f_per_fra(tmp_uv, tmp_t);

        auto it = find_if(feature.begin(), feature.end(), [tmp_id](const FeaturePerId &it)
                          {
            return it.feature_id == tmp_id;
                          });
        //如果没有则新建一个，并添加这图像帧
        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(tmp_id));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        //有的话把图像帧添加进去
         else if (it->feature_id == tmp_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
        }

    }

    // cout << " receive feature_msg" << endl;
    // cout << feature.size() << endl;

}

void cloud_callback(const sensor_msgs::PointCloudConstPtr &cloud_msg)
{

    int tmp_id;
    Vector3d tmp_p;

    for (unsigned int i = 0; i < cloud_msg->points.size(); i++)
    {
        tmp_p(0) = cloud_msg->points[i].x;
        tmp_p(1) = cloud_msg->points[i].y;
        tmp_p(2) = cloud_msg->points[i].z;
        tmp_id = cloud_msg->channels[0].values[i];

        auto it = find_if(feature.begin(), feature.end(), [tmp_id](const FeaturePerId &it)
                          {
            return it.feature_id == tmp_id;
                          });
        //如果没有则新建一个，并添加这图像帧
        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(tmp_id));
            feature.back().add_gt( tmp_p );
        }
        //有的话把图像帧添加进去
         else if (it->feature_id == tmp_id)
        {
            it->add_gt( tmp_p );
        }
    }
    // cout << " receive cloud_msg" << endl;

}

void lodom_callback(const nav_msgs::OdometryPtr &lodom_msg)
{
    double tmp_t;
    Vector3d tmp_p;
    Quaterniond tmp_q;
    tmp_t = lodom_msg->header.stamp.toSec();
    tmp_p(0) = lodom_msg->pose.pose.position.x;
    tmp_p(1) = lodom_msg->pose.pose.position.y;
    tmp_p(2) = lodom_msg->pose.pose.position.z;
    tmp_q.w() = lodom_msg->pose.pose.orientation.w;
    tmp_q.x() = lodom_msg->pose.pose.orientation.x;
    tmp_q.y() = lodom_msg->pose.pose.orientation.y;
    tmp_q.z() = lodom_msg->pose.pose.orientation.z;

    // cout << " receive lodom " << endl;
    // cout << lpose.size() << endl;

    lpose.push_back( Pose(tmp_p, tmp_q, tmp_t) );
}


int main ( int argc, char **argv )
{
    //ROS初始化，设置句柄n
    ros::init( argc, argv, "calibr" );
    ros::NodeHandle n( "~" );
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    readParameters(n);

    ros::Subscriber sub_vodom = n.subscribe("/vins_estimator/odometry", 2000, vodom_callback);
    ros::Subscriber sub_image = n.subscribe("/vins_estimator/feature", 2000, feature_callback);
    ros::Subscriber sub_cloud = n.subscribe("/vins_estimator/margin_cloud", 2000, cloud_callback);
    ros::Subscriber sub_lodom = n.subscribe("/integrated_to_init", 2000, lodom_callback);

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        if( rec_count > 100 ) //一秒内没有受到消息 
            break;

        if( bflag == 1 ) //开始接受
            rec_count++;
        
        loop_rate.sleep();
    }

    vector<pair<Pose, Pose>> AB;
    Pose res;

    getAB( AB, vpose, lpose );
    cout << "here" << endl;
    HandEye( res, AB );
    cout << res.getSE3().matrix() << endl;
    cout << AB.size() << endl;

    double para[6], position[MAX_POINT][3], time = 0;
    Eigen::Matrix<double, 6, 1> se3 = res.getSE3().log();
    for(int i = 0; i < 6; ++i) para[i] = se3(i, 0);

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    // loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    ceres::LocalParameterization *local_parameterization = new PoseParameterization();
    problem.AddParameterBlock(para, 6, local_parameterization);
    problem.AddParameterBlock(&time, 1);

    int index, sect = feature.size() / MAX_POINT;
    auto iter = feature.begin();
    for ( int i = 0; i < MAX_POINT; i++ )
    {
        for(int j = 0; j < 3; j++) position[i][j] = iter->gt_p(j);
        problem.AddParameterBlock(position[i], 3);
        index = iter->feature_per_frame.size() / 3;
        ProjectionFactor *f = new ProjectionFactor(iter->feature_per_frame[index], iter->gt_p);
        problem.AddResidualBlock(f, loss_function, para, position[i], &time);
        iter += sect;
    }

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.BriefReport() << endl;
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> tcl(para);
    cout << Sophus::SE3::exp(tcl).matrix() << endl;
    cout << time << endl;

    return 0;
}