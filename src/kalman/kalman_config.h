//
// Created by david on 10.10.18.
//

#ifndef WRAPPER_KALMAN_CONFIG_H
#define WRAPPER_KALMAN_CONFIG_H

#include "kalman_filter.h"

namespace cpppc{

  template<
        int StateDim
      , int MeasDim
      , int CtlDim = 0
      , typename T = double>
  struct bbox_tracking_matrices{

    Eigen::Matrix<T, StateDim, StateDim> transition_matrix = Eigen::Matrix<T, StateDim, StateDim>::Identity();
    Eigen::Matrix<T, StateDim, MeasDim> measurement_matrix = Eigen::Matrix<T, StateDim, MeasDim>::Zero();
    Eigen::Matrix<T, StateDim, CtlDim>      control_matrix = Eigen::Matrix<T, StateDim, CtlDim>::Zero();

    // Transition Matrix for bbox smoothing
    bbox_tracking_matrices()
    {
      transition_matrix  << 1, 0, 1, 0,
                            0, 1, 0, 1,
                            0, 0, 1, 0,
                            0, 0, 0, 1;

      measurement_matrix << 1, 0,
                            0, 1;
    };
  };

  template<
        int StateDim
      , int MeasDim
      , int CtlDim = 0
      , typename T = double>
  struct odometry_filter_matrices{

    Eigen::Matrix<T, StateDim, StateDim> transition_matrix = Eigen::Matrix<T, StateDim, StateDim>::Identity();
    Eigen::Matrix<T, StateDim, MeasDim> measurement_matrix = Eigen::Matrix<T, StateDim, MeasDim>::Zero();
    Eigen::Matrix<T, StateDim, CtlDim>      control_matrix = Eigen::Matrix<T, StateDim, CtlDim>::Zero();

    // Transition Matrix for velocity smoothing
    odometry_filter_matrices()
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
