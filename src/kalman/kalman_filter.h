//
// Created by david on 06.10.18.
//

#ifndef WRAPPER_KALMAN_FILTER_H
#define WRAPPER_KALMAN_FILTER_H

#include <eigen3/Eigen/Dense>

template<int StateDim, int MeasDim, int CtlDim>
class Kalman_Filter {

public:
  Kalman_Filter() :
    pre_state(pre_state.Zero())
  , post_state(post_state.Zero())
  , transition_mtx(transition_mtx.Identity())
  , ctl_mtx(ctl_mtx.Zero())
  , pre_process_cov(pre_process_cov.Zero())
  , post_process_cov(post_process_cov.Zero())
  , process_noise(process_noise.Identity())
  , meas_mtx(meas_mtx.Zero())
  , meas_noise(meas_noise.Zero())
  , gain(gain.Zero())
  , residual(residual.Zero())
  {};

  // Normal KF, but possibility to extend to EKF

  void set_transition_mtx(const Eigen::Matrix<double, StateDim, StateDim> transition_mtx)
  {
    transition_mtx = transition_mtx;
  };

  void set_ctl_mtx(const Eigen::Matrix<double, StateDim, CtlDim> ctl_mtx)
  {
    ctl_mtx = ctl_mtx;
  };

  void set_meas_mtx(const Eigen::Matrix<double, MeasDim,  StateDim> meas_mtx)
  {
    meas_mtx = meas_mtx;
  };

  void predict(const Eigen::Matrix<double, CtlDim, 1> ctl_vec = Eigen::Matrix<double, CtlDim, 1>::Zero())
  {
    // Use process model to predict state at the next time step
    // x' = Fx [+ Bu]
    pre_state = transition_mtx * post_state;
    // Default prediction doesn't use control parameters
    if(ctl_vec != ctl_vec.Zero())
    {
      pre_state += ctl_mtx * ctl_vec;
    }

    // Adjust belief to account for the uncertainty in prediction
    // P' = FPFt + Q
    pre_process_cov = transition_mtx * post_process_cov * transition_mtx.transpose() + process_noise;
  };

  void update(const Eigen::Matrix<double, MeasDim, 1>   meas_vec)
  {
    // Compute residual between prediction and measurement
    // y = z - Hx'
    residual = meas_vec - meas_mtx * pre_state;

    auto temp = pre_process_cov * meas_mtx.transpose();

    // Compute scaling factor based on accuracy of both prediction and measurement
    // K = P'Ht (HP'Ht + R)^-1
    gain = temp *  (meas_mtx * temp + meas_noise).mtx().inverse();

    // Set state according to scaling factor (gain)
    // x = x' + Ky
    post_state = pre_state + gain * residual;

    // Update belief in the state based on measurement certainty
    // P = (I - KH)P'
    post_process_cov = pre_process_cov - gain * meas_mtx * pre_process_cov;
  };

  Eigen::Matrix<double, StateDim, 1>          post_state;

private:
  Eigen::Matrix<double, StateDim, 1>          pre_state;
  Eigen::Matrix<double, StateDim, StateDim>   transition_mtx;
  Eigen::Matrix<double, StateDim, StateDim>   pre_process_cov;
  Eigen::Matrix<double, StateDim, StateDim>   post_process_cov;
  Eigen::Matrix<double, StateDim, StateDim>   process_noise;
  Eigen::Matrix<double, StateDim, CtlDim>     ctl_mtx;

  Eigen::Matrix<double, MeasDim,  StateDim>   meas_mtx;
  Eigen::Matrix<double, MeasDim,  MeasDim>    meas_noise;
  Eigen::Matrix<double, StateDim, MeasDim>    gain;
  Eigen::Matrix<double, MeasDim,  1>          residual;

};

#endif //WRAPPER_KALMAN_FILTER_H