//
// Created by david on 06.10.18.
//

#ifndef WRAPPER_KALMAN_FILTER_H
#define WRAPPER_KALMAN_FILTER_H

#include <eigen3/Eigen/Dense>

namespace cpppc
{

  template<
      int StateDim
    , int MeasDim
    , int CtlDim = 0
    , typename T = double >

  class Kalman_Filter {

    using self_t  = Kalman_Filter<StateDim, MeasDim, CtlDim, T>;
    using value_t = T;

  public:
    Kalman_Filter() :
          pre_state(pre_state.Zero())
        , post_state(post_state.Zero())
        , transition_mtx(transition_mtx.Identity())
        , pre_process_cov(pre_process_cov.Zero())
        , post_process_cov(post_process_cov.Zero())
        , process_noise(process_noise.Identity())
        , ctl_mtx(ctl_mtx.Zero())
        , meas_mtx(meas_mtx.Zero())
        , meas_noise(meas_noise.Zero())
        , gain(gain.Zero())
        , residual(residual.Zero())
        {};

    Kalman_Filter(const Eigen::Matrix<value_t, StateDim, StateDim> transition_matrix
        , const Eigen::Matrix<value_t, MeasDim, StateDim> measurement_matrix  = Eigen::Matrix<value_t, MeasDim, StateDim>::Zero()
        , const Eigen::Matrix<value_t, StateDim, CtlDim> control_matrix       = Eigen::Matrix<value_t, StateDim, CtlDim>::Zero())
        : Kalman_Filter()
    {
      this->set_transition_mtx(transition_matrix);
      this->set_meas_mtx(measurement_matrix);
      this->set_ctl_mtx(control_matrix);
    }

    // No ownership
    // |
    // -> defaults for Destructor, Copy/Move Constructor, and for Copy/Move Constructor/Assignment

    ~Kalman_Filter()                          = default;
    Kalman_Filter(const self_t &)             = default;
    Kalman_Filter(self_t &&)                  = default;
    self_t & operator=(const self_t &)        = default;
    self_t & operator=(self_t &&)             = default;



    // Normal KF, but possibility to extend to EKF

    void set_transition_mtx( const Eigen::Matrix<value_t, StateDim, StateDim> transition_mtx ) {
      this->transition_mtx = transition_mtx;
    };

    void set_ctl_mtx( const Eigen::Matrix<value_t, StateDim, CtlDim> ctl_mtx ) {
      this->ctl_mtx = ctl_mtx;
    };

    void set_meas_mtx( const Eigen::Matrix<value_t, MeasDim, StateDim> meas_mtx ) {
      this->meas_mtx = meas_mtx;
    };

    void set_process_noise(const Eigen::Matrix<value_t, StateDim, StateDim> process_noise)
    {
      this->process_noise = process_noise;
    };

    void set_process_cov(const Eigen::Matrix<value_t, StateDim, StateDim> post_process_cov)
    {
      this->post_process_cov = post_process_cov;
    };

    void set_measurement_noise(const Eigen::Matrix<value_t, MeasDim, MeasDim> meas_noise)
    {
      this->meas_noise = meas_noise;
    };

    void predict( const Eigen::Matrix<value_t, CtlDim, 1> ctl_vec = Eigen::Matrix<value_t, CtlDim, 1>::Zero()) {
      // Use process model to predict state at the next time step
      // x' = Fx [+ Bu]
      pre_state = transition_mtx * post_state;
      // Default prediction doesn't use control parameters
      if ( ctl_vec != ctl_vec.Zero()) {
        pre_state += ctl_mtx * ctl_vec;
      }

      // Adjust belief to account for the uncertainty in prediction
      // P' = FPFt + Q
      pre_process_cov = transition_mtx * post_process_cov * transition_mtx.transpose() + process_noise;
    };

    void update( const Eigen::Matrix<value_t, MeasDim, 1> meas_vec ) {
      // Compute residual between prediction and measurement
      // y = z - Hx'
      residual = meas_vec - meas_mtx * pre_state;

      auto temp = pre_process_cov * meas_mtx.transpose();

      // Compute scaling factor based on accuracy of both prediction and measurement
      // K = P'Ht (HP'Ht + R)^-1
      gain = temp * ( meas_mtx * temp + meas_noise ).inverse();

      // Set state according to scaling factor (gain)
      // x = x' + Ky
      post_state = pre_state + gain * residual;

      // Update belief in the state based on measurement certainty
      // P = (I - KH)P'
      post_process_cov = pre_process_cov - gain * meas_mtx * pre_process_cov;
    };

  public:
    Eigen::Matrix<value_t, StateDim, 1>           pre_state;
    Eigen::Matrix<value_t, StateDim, 1>           post_state;

  private:
    Eigen::Matrix<value_t, StateDim, StateDim>    transition_mtx;
    Eigen::Matrix<value_t, StateDim, StateDim>    pre_process_cov;
    Eigen::Matrix<value_t, StateDim, StateDim>    post_process_cov;
    Eigen::Matrix<value_t, StateDim, StateDim>    process_noise;
    Eigen::Matrix<value_t, StateDim, CtlDim>      ctl_mtx;

    Eigen::Matrix<value_t, MeasDim, StateDim>     meas_mtx;
    Eigen::Matrix<value_t, MeasDim, MeasDim>      meas_noise;
    Eigen::Matrix<value_t, StateDim, MeasDim>     gain;
    Eigen::Matrix<value_t, MeasDim, 1>            residual;

  };

} // cpppc

#endif //WRAPPER_KALMAN_FILTER_H