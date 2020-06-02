#include "project_factor.h"

vector<Pose> vpose;
vector<Pose> lpose;

// bool ProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
// {
//     Eigen::Map<const Eigen::Matrix<double, 6, 1>> tlc(*parameters);
//     Eigen::Map<const Eigen::Vector3d> pj(*(parameters + 1));
//     double time = parameters[2][0];

//     Eigen::Map<Eigen::Vector2d> residual(residuals);

//     Pose twl, betw;
//     auto r_pose = find_pose(lpose, pts_i.time + time );
//     twl = r_pose[0];
//     betw = r_pose[1];

//     if (twl.time == -1)
//         return false;

//     Vector3d p0 = twl.getSE3() * Sophus::SE3::exp(tlc).inverse() * pj;
//     Vector3d projected_pj = Sophus::SE3::exp(tlc) * p0;
//     residual = pts_i.uv - (projected_pj / projected_pj.z()).head<2>();

//     if ( jacobians )
//     {
//         Eigen::Matrix<double, 2, 3> reduce(2, 3);
//         reduce << -1. / projected_pj.z(), 0, projected_pj.x() / (projected_pj.z() * projected_pj.z()),
//                     0, -1. / projected_pj.z(), projected_pj.y() / (projected_pj.z() * projected_pj.z());
//         if ( jacobians[0] )
//         {
//             Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> jacobian_tlc(jacobians[0]);
//             Eigen::Matrix<double, 3, 6> jaco_tlc;
//             jaco_tlc.leftCols<3>() = Matrix3d::Identity();
//             jaco_tlc.rightCols<3>() = -1 * Sophus::SO3::hat(projected_pj);
//             jacobian_tlc = reduce * jaco_tlc;
//         }

//         if ( jacobians[1] )
//         {
//             Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_p(jacobians[1]);
//             Matrix3d jaco_p = Sophus::SE3::exp(tlc).rotation_matrix() * twl.getSO3().matrix();
//             jacobian_p = reduce * jaco_p;
//         }

//         if( jacobians[2] )
//         {
//             Eigen::Map<Eigen::Vector2d> jacobian_t(jacobians[2]);
//             Eigen::Matrix<double, 3, 6> jaco_t1;
//             Eigen::Matrix<double, 6, 1> jaco_t2;
//             jaco_t1.leftCols<3>() = Sophus::SE3::exp(tlc).rotation_matrix();
//             jaco_t1.rightCols<3>() = -1 * Sophus::SE3::exp(tlc).rotation_matrix() * Sophus::SO3::hat(p0);
//             jaco_t2 = betw.getSE3().log() / betw.time ;
//             jacobian_t = reduce * jaco_t1 * jaco_t2;
//         }
//     }

//     return true;
// }

bool ProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> tcl(*parameters);
    Eigen::Map<const Eigen::Vector3d> pj(*(parameters + 1));
    double time = parameters[2][0];

    Eigen::Map<Eigen::Vector2d> residual(residuals);

    auto Tcl = Sophus::SE3::exp(tcl);

    Pose  betw;
    auto r_pose = find_pose(lpose, pts_i.time - time );
    if (r_pose[0].time == -1)
        return false;
    auto Twl = r_pose[0].getSE3();
    betw = r_pose[1];

    

    Vector3d p0 = Twl * Tcl * pj;
    Vector3d projected_pj = Tcl.inverse() * p0;
    residual = pts_i.uv - (projected_pj / projected_pj.z()).head<2>();

    if ( jacobians )
    {
        Eigen::Matrix<double, 2, 3> reduce(2, 3);
        reduce << -1. / projected_pj.z(), 0, projected_pj.x() / (projected_pj.z() * projected_pj.z()),
                    0, -1. / projected_pj.z(), projected_pj.y() / (projected_pj.z() * projected_pj.z());
        if ( jacobians[0] )
        {
            Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> jacobian_tcl(jacobians[0]);
            Eigen::Matrix<double, 3, 6> jaco_tcl1, jaco_tcl2;
            jaco_tcl1.leftCols<3>() = Tcl.inverse().rotation_matrix() * Twl.rotation_matrix();
            jaco_tcl1.rightCols<3>() = -1 * jaco_tcl1.leftCols<3>() * Sophus::SO3::hat(Tcl * pj);
            jaco_tcl2.leftCols<3>() = Tcl.inverse().rotation_matrix();
            jaco_tcl2.rightCols<3>() = -1 * jaco_tcl2.leftCols<3>() * Sophus::SO3::hat(p0);
            jacobian_tcl = reduce * (jaco_tcl1 - jaco_tcl2);
        }

        if ( jacobians[1] )
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_p(jacobians[1]);
            Matrix3d jaco_p = Tcl.inverse().rotation_matrix() * Twl.rotation_matrix() * Tcl.rotation_matrix();
            jacobian_p = reduce * jaco_p;
        }

        if( jacobians[2] )
        {
            Eigen::Map<Eigen::Vector2d> jacobian_t(jacobians[2]);
            Eigen::Matrix<double, 3, 6> jaco_t1;
            Eigen::Matrix<double, 6, 1> jaco_t2;
            jaco_t1.leftCols<3>() = Tcl.inverse().rotation_matrix();
            jaco_t1.rightCols<3>() = -1 * jaco_t1.leftCols<3>() * Sophus::SO3::hat(p0);
            jaco_t2 = betw.getSE3().log() / betw.time ;
            jacobian_t = reduce * jaco_t1 * jaco_t2;
        }
    }

    return true;
}