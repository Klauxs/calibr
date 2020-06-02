#ifndef PROJECT_FACTOR_H
#define PROJECT_FACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "data_class.h"

extern vector<Pose> vpose;
extern vector<Pose> lpose;

class ProjectionFactor : public ceres::SizedCostFunction<2, 6, 3, 1>
{
  public:
    ProjectionFactor(const FeaturePerFrame _pts_i, const Eigen::Vector3d &_pts_j):
    pts_i(_pts_i), pts_j(_pts_j){}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    FeaturePerFrame pts_i;
    Eigen::Vector3d pts_j;
    // vector<Pose> lpose;
};

#endif