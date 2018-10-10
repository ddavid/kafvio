//
// Created by david on 08.10.18.
//

#ifndef WRAPPER_KALMAN_TRACKER_H
#define WRAPPER_KALMAN_TRACKER_H

#include "kalman_filter.h"
#include "../yolo_v2_class.hpp"
#include <list>

namespace cpppc {

  template<
        int StateDim
      , int MeasDim
      , int CtlDim = 0
      , typename T = double >
  struct Tracking_Kafi{

    using kafi_t = Kalman_Filter<StateDim, MeasDim, CtlDim, T>;

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
        }

    bool          _removal_flag;
    unsigned int  _track_id;
    kafi_t        _tracking_kalman_filter;
  };

  template<
        int StateDim
      , int MeasDim
      , int CtlDim = 0
      , typename T = double>
  class BBox_Tracker{

    using kafi_t = Tracking_Kafi<StateDim, MeasDim, CtlDim, T>;

    // const because at first, we won't modify the bbox coords
    BBox_Tracker(const std::vector<bbox_t> cur_bbox_vec) {};

    // Optional control parameter, makes prediction for all kafis
    void predict(){};

    // Updates kafis based on according bboxes (track_id) and puts updated coords in bboxes
    void update_bbox_vec(std::vector<bbox_t> cur_bbox_vec) {};

    // Remove kafis for bboxes that aren't being tracked anymore
    void clear_old_trackers()
    {
      // search for bboxes with removal_flag
      // erase-remove idiom
      _prev_bbox_vec.erase(
            std::remove(_prev_bbox_vec.begin(), _prev_bbox_vec.end(), [](const kafi_t & kafi){ return kafi._removal_flag; })
          , _prev_bbox_vec.end());
    }

  private:

    std::vector<bbox_t> _prev_bbox_vec;
    std::list<kafi_t>   _tracking_kafi_list;
    Eigen::Matrix<T, StateDim, StateDim>
  };
  // Class to wrap a kalman tracker
  // Map obj_ids to instances of kfs coming from a kf factory (lol!) -> Just have the transition matrix etc. as private members here...
  // All have the same transition matrix, meas_mtx etc.) but different starting states
  /*
   * State x := (x, y, dx, dy)^T
   * Measurements z := (x, y)^T
   *
   * Goal of KF is to smooth the trend of the bboxes
   */

  /*
   * Alternativ:
   *
   * State x := (h, v, a)^T
   * Measurement z := (h, v, a)^T
   *
   */

  /*
   * Alternativ: KFs for smoothing and for predicting the velocity -> v from vis. od. fused with accell from IMU
   *
   * State x := (v, a)^T
   * Measurement z := (v, a)^T
   *
   */
}
#endif //WRAPPER_KALMAN_TRACKER_H
