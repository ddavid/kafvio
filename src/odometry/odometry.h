//
// Created by david on 10.10.18.
//

#ifndef WRAPPER_ODOMETRY_H
#define WRAPPER_ODOMETRY_H

#include "../yolo_v2_class.hpp"
#include "../kalman/kalman_filter.h"
#include <eigen3/Eigen/Dense>
#include <wrapper/detector-wrapper.hpp>

// Distance Estimation
#include "distance_calc.h"

namespace cpppc {

  template<
        int StateDim
      , int MeasDim
      , int CtlDim = 0
      , typename T = double >
  class Odometry {

    using self_t  = Odometry<StateDim, MeasDim, CtlDim, T>;
    using value_t = T;
    using kafi_t  = cpppc::Kalman_Filter<StateDim, MeasDim, CtlDim, value_t>;

  public:

    Odometry() = default;

    // Copying cur_bbox_vec on each use -> Try to use std::weak_pointer along with std::shared_pointer on other side
    Odometry(
          const std::vector<bbox_t> cur_bbox_vec
        , Eigen::Matrix<value_t, StateDim, StateDim> transition_matrix = Eigen::Matrix<value_t, StateDim, StateDim>::Identity()
        , Eigen::Matrix<value_t, MeasDim, StateDim> measurement_matrix = Eigen::Matrix<value_t, MeasDim, StateDim>::Zero()
        , Eigen::Matrix<value_t, StateDim, CtlDim> control_matrix = Eigen::Matrix<value_t, StateDim, CtlDim>::Zero())
        :
          _velocity_kafi(transition_matrix, measurement_matrix)
        {
          std::vector<bbox_t> tmp_vec(cur_bbox_vec);
          std::sort(
                tmp_vec.begin()
              , tmp_vec.end()
              , [](const bbox_t & fst_bbox, const bbox_t & snd_bbox) {return fst_bbox.track_id < snd_bbox.track_id;});
          this->_prev_bbox_vec = std::move(tmp_vec);
        };

    ~Odometry() = default;

    Odometry( const self_t & ) = default;

    Odometry( self_t && ) = default;

    self_t &operator=( const self_t & ) = default;

    self_t &operator=( self_t && ) = default;

    // Accumulate difference in distances for bboxes matched by track_id
    // -> return Average distance / time_step
    const double calc_velocity( const std::vector<bbox_t> cur_bbox_vec ) const
    {
      // Sort current bbox_vec by track_id
      std::vector<bbox_t> cur_tmp_vec(cur_bbox_vec);
      std::sort(
          cur_tmp_vec.begin()
          , cur_tmp_vec.end()
          , [](const bbox_t & fst_bbox, const bbox_t & snd_bbox) {return fst_bbox.track_id < snd_bbox.track_id;});

      // Get intersection based on track_id
      std::vector<bbox_t> matched_track_ids;
      std::vector<bbox_t> set_intersection( _prev_bbox_vec.begin(), _prev_bbox_vec.end()
                               , cur_tmp_vec.begin(), cur_tmp_vec.end()
                               , std::back_inserter(matched_track_ids)
                               , [](const bbox_t & fst_bbox, const bbox_t & snd_bbox )
                               {
                                  return fst_bbox.track_id < snd_bbox.track_id;
                               });

      const double time_step(_time_step);
      const double acc_vel(std::accumulate(
                cur_bbox_vec.begin()
                , cur_bbox_vec.end()
                , 0
                , [time_step](const double & acc_dist, const bbox_t & bbox)
                {
                  return ( acc_dist + distance_estimation( bbox, Distance_Strategy::CONE_HEIGHT)) / time_step;
                }));


      // Update previous sorted bbox vector with current sorted bbox vector
      this->update_bboxes(std::move(cur_tmp_vec));

      // Average velocity over # of bboxes
      return acc_vel / cur_bbox_vec.size();
    };

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
    void update_bboxes( const std::vector<bbox_t> bbox_vec, double time_step = 0)
    {
      // Update time_step
      if(time_step != 0) {
        this->_time_step = time_step;
        this->update_time_step(time_step);
      }
      // Set new bbox_vec
      this->_prev_bbox_vec = bbox_vec;
    };

    // Without updating time_step, we assume acceleration is given scaled to the size of the time_step
    void update_time_step(double time_step)
    {
      _velocity_kafi.transition_matrix(0, 1) = static_cast<T>(time_step);
    }

  private:

    kafi_t                _velocity_kafi;
    std::vector<bbox_t>   _prev_bbox_vec;
    double                _time_step = 1;
  };
} // cpppc
#endif //WRAPPER_ODOMETRY_H
