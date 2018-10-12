//
// Created by david on 10.10.18.
//

#ifndef WRAPPER_KALMAN_CONFIG_H
#define WRAPPER_KALMAN_CONFIG_H

#include "kalman_filter.h"

namespace cpppc{

  template<typename T = double>
  struct Bbox_Tracking_Matrices{

    Eigen::Matrix<T, 4, 4> transition_matrix = Eigen::Matrix<T, 4, 4>::Identity();
    Eigen::Matrix<T, 2, 4> measurement_matrix = Eigen::Matrix<T, 2, 4>::Zero();
    Eigen::Matrix<T, 4, 0>      control_matrix = Eigen::Matrix<T, 4, 0>::Zero();

    // Transition Matrix for bbox smoothing
    Bbox_Tracking_Matrices()
    {
      transition_matrix  << 1, 0, 1, 0,
                            0, 1, 0, 1,
                            0, 0, 1, 0,
                            0, 0, 0, 1;

      measurement_matrix << 1, 0, 0, 0,
                            0, 1, 0, 0;
    };
  };

  template<typename T = double>
  struct Odometry_Filter_Matrices{

    Eigen::Matrix<T, 2, 2> transition_matrix  = Eigen::Matrix<T, 2, 2>::Identity();
    Eigen::Matrix<T, 2, 2> measurement_matrix = Eigen::Matrix<T, 2, 2>::Zero();
    Eigen::Matrix<T, 2, 0>     control_matrix = Eigen::Matrix<T, 2, 0>::Zero();

    // Transition Matrix for velocity smoothing
    Odometry_Filter_Matrices()
    {
      // upper right value depends on time step
      transition_matrix  << 1, 1,
                            0, 1;

      measurement_matrix << 1, 0,
                            0, 1;
    };
  };

} // cpppc

#endif //WRAPPER_KALMAN_CONFIG_H
