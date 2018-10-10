//
// Created by david on 10.10.18.
//

#ifndef WRAPPER_ODOMETRY_H
#define WRAPPER_ODOMETRY_H

#include "../yolo_v2_class.hpp"
#include "../kalman/kalman_filter.h"
#include <eigen3/Eigen/Dense>

namespace cpppc {

  template<
      int StateDim, int MeasDim, int CtlDim = 0, typename T   = double>

  class Odometry {

    using self_t  = Odometry<StateDim, MeasDim, CtlDim, T>;
    using value_t = T;

  public:

    Odometry() = default;

    // Copying cur_bbox_vec on each use -> Try to use std::weak_pointer along with std::shared_pointer on other side
    Odometry(
          const std::vector<bbox_t> cur_bbox_vec
        , double time_step
        , Eigen::Matrix<value_t, StateDim, StateDim> transition_matrix = Eigen::Matrix<value_t, StateDim, StateDim>::Identity()
        , Eigen::Matrix<value_t, MeasDim, StateDim> measurement_matrix = Eigen::Matrix<value_t, MeasDim, StateDim>::Zero()
        , Eigen::Matrix<value_t, StateDim, CtlDim> control_matrix = Eigen::Matrix<value_t, StateDim, CtlDim>::Zero())
        :
          _velocity_kafi(transition_matrix, measurement_matrix)
        , _prev_bbox_vec(cur_bbox_vec)
        , _time_step(time_step) {};

    ~Odometry() = default;

    Odometry( const self_t & ) = default;

    Odometry( self_t && ) = default;

    self_t &operator=( const self_t & ) = default;

    self_t &operator=( self_t && ) = default;

    // Accumulate difference in distances for bboxes matched by obj_id
    // -> return Average distance / time_step
    const double calc_velocity( const std::vector<bbox_t> cur_bbox_vec ) const;

    /* Calculate velocity using KaFi
     * Performs the update step and returns the velocity from the post_state
     */
    const double calc_filtered_velocity(
          const std::vector<bbox_t> cur_bbox_vec
        , Eigen::Matrix<value_t, MeasDim, 1> meas_vec
        , double time_step );

    /* Replace old bbox vector
     * Adjust tansition_matrix if necessary and make prediction step
     */
    void update_bboxes( const std::vector<bbox_t> bbox_vec, double time_step );

    void update_time_step(double time_step)
    {
      _velocity_kafi.transition_matrix(0, 1) = static_cast<T>(time_step);
    }

  private:

    cpppc::Kalman_Filter<StateDim, MeasDim, CtlDim, value_t> _velocity_kafi;
    std::vector<bbox_t> _prev_bbox_vec;
    double _time_step = 1;
  };
} // cpppc
#endif //WRAPPER_ODOMETRY_H
