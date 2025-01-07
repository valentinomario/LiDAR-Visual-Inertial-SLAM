/**
The projection_td_factor implements a projection factor with support for Time Delay (Td) and Rolling Shutter Effects:

Corrects errors caused by temporal misalignment between IMU and camera measurements.

Models the effect of rolling shutter, which introduces temporal variations in pixel rows during image capture.

**/

#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"

// Support for incomplete synchronization of imu-camera timestamps and Rolling shutter cameras
class ProjectionTdFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1, 1>
{
public:
    ProjectionTdFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
                       const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
                       const double _td_i, const double _td_j, const double _row_i, const double _row_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;// Coordinates of the corner point in the normalized plane
    Eigen::Vector3d velocity_i, velocity_j;// The velocity of the corner point in the normalized plane
    double td_i, td_j;// Time synchronization error used when processing IMU data
    Eigen::Matrix<double, 2, 3> tangent_base;
    double row_i, row_j;// The ordinate of the corner point image coordinates
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};
