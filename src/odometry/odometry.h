//
// Created by david on 10.10.18.
//

#ifndef WRAPPER_ODOMETRY_H
#define WRAPPER_ODOMETRY_H

#include "../wrapper/opencv_utils.hpp"
#include "../wrapper/detector-wrapper.hpp"
#include "../kalman/kalman_filter.h"
#include <eigen3/Eigen/Dense>
#include <wrapper/detector-wrapper.hpp>
#include <cmath>

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
        , Eigen::Matrix<value_t, StateDim, CtlDim> control_matrix      = Eigen::Matrix<value_t, StateDim, CtlDim>::Zero())
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
    const double calc_velocity( const std::vector<bbox_t> cur_bbox_vec )
    {
      int counter         = 0;
      double acc_distance = 0;
      double avg_distance;

      // Calculate distance Deltas for all track_id matches
      std::for_each(
            cur_bbox_vec.begin()
          , cur_bbox_vec.end()
          , [&](const bbox_t & bbox)
          {
            auto prev_it = std::find_if(
                  _prev_bbox_vec.begin()
                , _prev_bbox_vec.end()
                , [bbox](const bbox_t & prev_bbox)
                {
                  return bbox.track_id == prev_bbox.track_id;
                });
            if(prev_it != _prev_bbox_vec.end())
            {
              ++counter;
              acc_distance += distance_estimation(*prev_it, Distance_Strategy::CONE_HEIGHT) - distance_estimation(bbox, Distance_Strategy::CONE_HEIGHT);
            }
          });
      // Average Distance Deltas
      avg_distance = ( acc_distance / counter);
      // Update previous sorted bbox vector with current sorted bbox vector
      this->update_bboxes(cur_bbox_vec);
      std::cout << "Average velocity: " << avg_distance << std::endl;
      // Average velocity
      const double avg_velocity = avg_distance / _time_step;
      if(!std::isnan(avg_velocity))
      {
        return avg_velocity;
      }
      else return 0.0;
    };

    /* Calculate velocity using KaFi
     * Performs the update step and returns the velocity from the post_state
     */
    const double calc_filtered_velocity(
          const std::vector<bbox_t> cur_bbox_vec
        , const double accell)
    {
      // Create measurement vector
      Eigen::Matrix<value_t, 2, 1> measurement;
      measurement << calc_velocity(cur_bbox_vec), static_cast<value_t >(accell);

      this->_velocity_kafi.update(measurement);
      std::cout << "Kafi pre state after update: " << this->_velocity_kafi.pre_state << std::endl;
      std::cout << "Kafi post state after update: " << this->_velocity_kafi.post_state << std::endl;
      const double filtered_velocity(this->_velocity_kafi.post_state(0, 0));

      std::cout << "Filtered velocity: " << filtered_velocity << std::endl;
      return filtered_velocity;
    };

    /* Replace old bbox vector
     * Adjust tansition_matrix if necessary and make prediction step
     */
    void update_bboxes( const std::vector<bbox_t> bbox_vec, double time_step = 1)
    {
      this->_velocity_kafi.predict();
      std::cout << "Predicted Odom Kafi: " << std::endl;
      std::cout << "Kafi pre state after predict: " << _velocity_kafi.pre_state << std::endl;
      std::cout << "Kafi post state after predict: " << _velocity_kafi.post_state << std::endl;

      // Update time_step
      if(time_step != 1) {
        this->_time_step = time_step;
        this->update_kafi_time_step(time_step);
      }
      // Set new bbox_vec
      this->_prev_bbox_vec = bbox_vec;
    };
    
    void set_time_step(const double time_step)
    {
      _time_step = time_step;
    }

    // Without updating time_step, we assume acceleration is given scaled to the size of the time_step
    void update_kafi_time_step(const double time_step)
    {
      _velocity_kafi.transition_mtx(0, 1) = static_cast<T>(time_step);
    }

  private:

    kafi_t                _velocity_kafi;
    std::vector<bbox_t>   _prev_bbox_vec;
    double                _time_step = 1;
  };
} // cpppc
#endif //WRAPPER_ODOMETRY_H
