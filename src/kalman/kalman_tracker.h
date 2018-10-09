//
// Created by david on 08.10.18.
//

#ifndef WRAPPER_KALMAN_TRACKER_H
#define WRAPPER_KALMAN_TRACKER_H

#include "../yolo_v2_class.hpp"

template <typename T>

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

#endif //WRAPPER_KALMAN_TRACKER_H
