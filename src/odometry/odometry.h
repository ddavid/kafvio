//
// Created by david on 10.10.18.
//

#ifndef WRAPPER_ODOMETRY_H
#define WRAPPER_ODOMETRY_H

#include "../yolo_v2_class.hpp"
#include "../kalman/kalman_filter.h"
#include <eigen3/Eigen/Dense>

template<
      int StateDim
    , int MeasDim
    , int CtlDim   = 0
    , typename T   = double >

class Odometry{

  typedef Odometry<StateDim, MeasDim, CtlDim, T>  self_t;
  typedef T                                       value_t;

public:

  Odometry() = default;

  Odometry( const std::vector<bbox_t> cur_bbox_vec
          , double time_step
          , Eigen::Matrix<T, StateDim, StateDim> transition_matrix = Eigen::Matrix<T, StateDim, StateDim>::Identity()
          , Eigen::Matrix<T, MeasDim, StateDim> measurement_matrix = Eigen::Matrix<T, MeasDim, StateDim>::Zero()
          , Eigen::Matrix<T, StateDim, CtlDim> control_matrix      = Eigen::Matrix<T, StateDim, CtlDim>::Zero())
          :
            _velocity_kafi( transition_matrix, measurement_matrix )
          , _prev_bbox_vec( cur_bbox_vec )
          , _time_step( time_step )
          {};

  ~Odometry()                            = default;
  Odometry(const self_t &)               = default;
  Odometry(self_t &&)                    = default;
  self_t & operator=(const self_t &)     = default;
  self_t & operator=(self_t &&)          = default;

  // Accumulate difference in distances for bboxes matched by obj_id
  // -> return Average distance / time_step
  const double calc_velocity( const std::vector<bbox_t> cur_bbox_vec ) const;

  /* Calculate velocity using KaFi
   * Performs the update step and returns the velocity from the post_state
   */
  const double calc_filtered_velocity( const std::vector<bbox_t> cur_bbox_vec, Eigen::Matrix<T, MeasDim , 1> meas_vec, double time_step );

  /* Replace old bbox vector
   * Adjust tansition_matrix if necessary and make prediction step
   */
  void update_bboxes( const std::vector<bbox_t> bbox_vec, double time_step );

private:

  cpppc::Kalman_Filter< StateDim, MeasDim, CtlDim, T >   _velocity_kafi;
  std::vector<bbox_t>                                    _prev_bbox_vec;
  double                                                 _time_step = 1;
};

#endif //WRAPPER_ODOMETRY_H
