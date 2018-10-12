//
// Created by david on 08.10.18.
//

#ifndef WRAPPER_KALMAN_TRACKER_H
#define WRAPPER_KALMAN_TRACKER_H

#include "kalman_filter.h"
#include "kalman_config.h"
#include "../wrapper/opencv_utils.hpp"
#include "../wrapper/detector-wrapper.hpp"
#include <list>
#include <algorithm>

namespace cpppc {

  template<
        int StateDim
      , int MeasDim
      , int CtlDim = 0
      , typename T = double >
  struct Tracking_Kafi{

    using kafi_t = Kalman_Filter<StateDim, MeasDim, CtlDim, T>;
    using self_t = Tracking_Kafi<StateDim, MeasDim, CtlDim, T>;

  /*
   * Wrapper Struct for Kalman Filters with tracking_ids and state initialization
   */
    Tracking_Kafi(
          const bbox_t & bbox
        , Eigen::Matrix<T, StateDim, StateDim> transition_matrix = Eigen::Matrix<T, StateDim, StateDim>::Identity()
        , Eigen::Matrix<T, StateDim, MeasDim> measurement_matrix = Eigen::Matrix<T, StateDim, MeasDim>::Zero()
        , Eigen::Matrix<T, StateDim, CtlDim> control_matrix = Eigen::Matrix<T, StateDim, CtlDim>::Zero())
        :
          _track_id(bbox.track_id)
        , _tracking_kalman_filter(transition_matrix, measurement_matrix, control_matrix)
        {
          _tracking_kalman_filter.post_state(0, 0) = bbox.x;
          _tracking_kalman_filter.post_state(1, 0) = bbox.y;

          _tracking_kalman_filter.set_process_noise(Eigen::Matrix<T, StateDim, StateDim>::Identity() * 0.0001);
          _tracking_kalman_filter.set_measurement_noise(Eigen::Matrix<T, MeasDim, MeasDim>::Identity() * 10);
          _tracking_kalman_filter.set_process_cov(Eigen::Matrix<T, StateDim, StateDim>::Identity() * 0.1);
        };

    ~Tracking_Kafi() = default;

    Tracking_Kafi( const self_t & ) = default;

    Tracking_Kafi( self_t && ) = default;

    self_t &operator=( const self_t & ) = default;

    self_t &operator=( self_t && ) = default;

    bool          _removal_flag = false;
    unsigned int  _track_id;
    kafi_t        _tracking_kalman_filter;
  };

  template<
        int StateDim
      , int MeasDim
      , int CtlDim = 0
      , typename T = double>
  class BBox_Tracker{

    using self_t     = BBox_Tracker<StateDim, MeasDim, CtlDim, T>;  
    using tracker_t  = Tracking_Kafi<StateDim, MeasDim, CtlDim, T>;
    using matrices_t = Bbox_Tracking_Matrices<T>;

  public:
    // const because at first, we won't modify the bbox coords
    BBox_Tracker(const std::vector<bbox_t> & cur_bbox_vec)
    : _prev_bbox_vec(cur_bbox_vec)
    {};

    ~BBox_Tracker() = default;

    BBox_Tracker( const self_t & ) = default;

    BBox_Tracker( self_t && ) = default;

    self_t &operator=( const self_t & ) = default;

    self_t &operator=( self_t && ) = default;

    // Optional control parameter, makes prediction for all kafis
    void predict( Eigen::Matrix<T, CtlDim, 1> control_vector = Eigen::Matrix<T, CtlDim, 1>::Zero())
    {
      std::for_each(
            _tracking_kafi_list.begin()
          , _tracking_kafi_list.end()
          , [control_vector](tracker_t & tracker)
          {
            tracker._tracking_kalman_filter.predict(control_vector);
          });
    };

    // Initialize kafis for all newly tracked bboxes
    void initialize_trackers(const std::vector<bbox_t> & cur_bbox_vec)
    {
      // Find all bboxes that appear in both current and previous vec
      std::for_each(
          cur_bbox_vec.begin()
          , cur_bbox_vec.end()
          , [&](const bbox_t & cur_bbox)
          {
            auto prev_it = std::find_if(
                _prev_bbox_vec.begin()
                , _prev_bbox_vec.end()
                , [cur_bbox](const bbox_t & prev_bbox)
                {
                  return cur_bbox.track_id == prev_bbox.track_id;
                });
            // Matched bbox found
            if(prev_it != _prev_bbox_vec.end())
            {
              // Initialize Kafi for newly tracked bboxes
              if(!std::any_of(
                  _tracking_kafi_list.begin(), _tracking_kafi_list.end()
                  , [cur_bbox](const tracker_t & kafi){ return cur_bbox.track_id == kafi._track_id;}))
              {
                tracker_t tracking_kafi(
                    cur_bbox
                    , _bbox_tracking_matrices.transition_matrix
                    , _bbox_tracking_matrices.measurement_matrix);
                _tracking_kafi_list.push_front(tracking_kafi);
              }

            }
          });
    }

    // Updates kafis based on according bboxes (track_id) and puts updated coords in bboxes
    void update_tracker(std::vector<bbox_t> & cur_bbox_vec)
    {
      this->initialize_trackers(cur_bbox_vec);

      // Update corresponding kafis with bbox coordinates
      std::for_each(
            cur_bbox_vec.begin()
          , cur_bbox_vec.end()
          , [&](bbox_t & cur_bbox)
          {
            // Perform update step for all trackers that are old enough
            if(cur_bbox.frames_counter > 2)
            {
              auto tracker_it = std::find_if(_tracking_kafi_list.begin(), _tracking_kafi_list.end(), [cur_bbox](const tracker_t & kafi)
              {
                  return cur_bbox.track_id == kafi._track_id;
              });
              // Update Kafi
              Eigen::Matrix<T, MeasDim, 1> measurement_vector;
              measurement_vector << cur_bbox.x, cur_bbox.y;
              tracker_it->_tracking_kalman_filter.update(measurement_vector);
              // Adjust bbox coords based on Kafi
              cur_bbox.x = tracker_it->_tracking_kalman_filter.post_state(0, 0);
              cur_bbox.y = tracker_it->_tracking_kalman_filter.post_state(1, 0);
            }
          });

      // Set current bbox_vec
      _prev_bbox_vec = cur_bbox_vec;
    };

    // Remove kafis for bboxes that aren't being tracked anymore
    void clear_old_trackers()
    {
      // search for bboxes with removal_flag
      // erase-remove idiom
      _prev_bbox_vec.erase(
            std::remove_if(
                  _prev_bbox_vec.begin()
                , _prev_bbox_vec.end()
                , [](const tracker_t & kafi)
                {
                    return kafi._removal_flag;
                }));
    }

  private:

    std::vector<bbox_t>    _prev_bbox_vec;
    std::list<tracker_t>   _tracking_kafi_list;
    matrices_t             _bbox_tracking_matrices;
  };
}
#endif //WRAPPER_KALMAN_TRACKER_H
