#include "pose_parameterization.h"

bool PoseParameterization::ComputeJacobian(const double *x, double *jacobian) const {
    ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6, 6);
    return true;
}

bool PoseParameterization::Plus(const double* x,
                               const double* delta,
                               double* x_plus_delta) const {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie(x);
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta_lie(delta);

    Sophus::SE3 T = Sophus::SE3::exp(lie);
    Sophus::SE3 delta_T = Sophus::SE3::exp(delta_lie);
    Eigen::Matrix<double, 6, 1> x_plus_delta_lie = (T * delta_T).log();

    for(int i = 0; i < 6; ++i) x_plus_delta[i] = x_plus_delta_lie(i, 0);

    return true;

}