#ifndef POSE_PARAMETERIZATION_H
#define POSE_PARAMETERIZATION_H

#include <ceres/ceres.h>
#include <sophus/se3.h>

class PoseParameterization : public ceres::LocalParameterization {
public:
    virtual bool Plus(const double* x,
                      const double* delta,
                      double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x,
                                 double* jacobian) const;
    virtual int GlobalSize() const { return 6; };
    virtual int LocalSize() const { return 6; };
};

#endif