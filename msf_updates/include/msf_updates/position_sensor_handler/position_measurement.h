/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 * Copyright (C) 2011-2012 Stephan Weiss, ASL, ETH Zurich, Switzerland
 * You can contact the author at <stephan dot weiss at ieee dot org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef POSITION_MEASUREMENT_HPP_
#define POSITION_MEASUREMENT_HPP_

#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>
#include <msf_core/eigen_utils.h>
#include <sensor_fusion_comm/PointWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

namespace msf_updates {
namespace position_measurement {
enum {
  nMeasurements = 6
};


enum fault_t {
  FAULT_NONE = 0,
  FAULT_MINOR,
  FAULT_SEVERE
};
fault_t _gpsFault;

// change this to set when
// the system will abort correcting a measurement
// given a fault has been detected
static const fault_t fault_lvl_disable = FAULT_SEVERE;

// for fault detection
// chi squared distribution, false alarm probability 0.0001
// see fault_table.py
// note skip 0 index so we can use degree of freedom as index
static const float BETA_TABLE[7] = {0,
            8.82050518214,
            12.094592431,
            13.9876612368,
            16.0875642296,
            17.8797700658,
            19.6465647819,
           };





/**
 * \brief A measurement as provided by a position sensor, e.g. Total Station, GPS.
 */
typedef msf_core::MSF_Measurement<
    nav_msgs::Odometry,
    Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState> PositionMeasurementBase;
struct PositionMeasurement : public PositionMeasurementBase {
 private:
  typedef PositionMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void MakeFromSensorReadingImpl(measptr_t msg) {

    Eigen::Matrix<double, nMeasurements,
        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();

    // Get measurement.
    z_p_ = Eigen::Matrix<double, 3, 1>(msg->pose.pose.position.x, 
                                       msg->pose.pose.position.y,
                                       0);

    z_v_ = Eigen::Matrix<double, 3, 1>(msg->twist.twist.linear.x, 
                                       msg->twist.twist.linear.y,
                                       0);

    // printf("vx = %.3f vy = %.3f\n", z_v_(0), z_v_(1));
    double s_zp;
    double s_zv;
    if (fixed_covariance_)  //  take fix covariance from reconfigure GUI
    {

      s_zp = n_zp_ * n_zp_;
      // s_zv = n_zv_ * n_zv_;
      
      R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zp, s_zp, 9999, s_zv, s_zv, 9999)
          .finished().asDiagonal();

    } else {  // Tke covariance from sensor.

      /*
      if(msg->covariance[0] > n_zp_) { 
        s_zp = msg->covariance[0] * msg->covariance[0];
      }else{
        s_zp = n_zp_ * n_zp_;
      }
      */
      s_zp = msg->pose.covariance[0] * msg->pose.covariance[0];
      s_zv = msg->pose.covariance[21] * msg->pose.covariance[21];
      // printf("position covariance %.4f", s_zp);

      R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zp, s_zp, 9999, s_zv, s_zv, 9999)
          .finished().asDiagonal();
      // R_.block<3, 3>(0, 0) = msf_core::Matrix3(&msg->covariance[0]);

      if (msg->header.seq % 100 == 0) {  // Only do this check from time to time.
        if (R_.block<3, 3>(0, 0).determinant() < -0.01)
          MSF_WARN_STREAM_THROTTLE(
              60, "The covariance matrix you provided for "
              "the position sensor is not positive definite");
      }
    }
  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double, 3, 1> z_p_;  /// Position measurement.
  Eigen::Matrix<double, 3, 1> z_v_;  /// Velocity measurement.
  double n_zp_;  /// Position measurement noise.
  // double n_zv_;  /// velocity measurement noise.

  bool fixed_covariance_;
  int fixedstates_;

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~PositionMeasurement() {
  }
  PositionMeasurement(double n_zp, bool fixed_covariance,
                      bool isabsoluteMeasurement, int sensorID, int fixedstates)
      : PositionMeasurementBase(isabsoluteMeasurement, sensorID),
        n_zp_(n_zp),
        fixed_covariance_(fixed_covariance),
        fixedstates_(fixedstates) {
  }
  virtual std::string Type() {
    return "position";
  }

  virtual void CalculateH(
      shared_ptr<EKFState_T> state_in,
      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& H) {
    const EKFState_T& state = *state_in;  // Get a const ref, so we can read core states.

    H.setZero();

    // Get rotation matrices.
    Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
        .conjugate().toRotationMatrix();

    // Preprocess for elements in H matrix.
    Eigen::Matrix<double, 3, 3> p_prism_imu_sk = Skew(
        state.Get<StateDefinition_T::p_ip>());

    // Get indices of states in error vector.
    enum {
      idxstartcorr_p_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::p>::value,
      idxstartcorr_v_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::v>::value,
      idxstartcorr_q_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::q>::value,
      idxstartcorr_p_pi_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::p_ip>::value,
    };

    bool fixed_p_pos_imu = (fixedstates_ & 1 << StateDefinition_T::p_ip);

    // Clear crosscorrelations.
    if (fixed_p_pos_imu)
      state_in->ClearCrossCov<StateDefinition_T::p_ip>();

    // Construct H matrix:
    // Position:
    H.block<3, 3>(0, idxstartcorr_p_) = Eigen::Matrix<double, 3, 3>::Identity();  // p
    H.block<1, 1>(0, idxstartcorr_p_+2)(0) = 0;
    H.block<3, 3>(0, idxstartcorr_q_) = -C_q.transpose() * p_prism_imu_sk;  // q

    H.block<3, 3>(0, idxstartcorr_p_pi_) =
        fixed_p_pos_imu ?
            Eigen::Matrix<double, 3, 3>::Zero() : (C_q.transpose()).eval();  //p_pos_imu_


    // Velocity:
    H.block<3, 3>(3, idxstartcorr_v_) = Eigen::Matrix<double, 3, 3>::Identity();  // v
    H.block<1, 1>(3, idxstartcorr_v_+2)(0) = 0;



  }

  /**
   * The method called by the msf_core to apply the measurement represented by this object.
   */
  virtual void Apply(shared_ptr<EKFState_T> state_nonconst_new,
                     msf_core::MSF_Core<EKFState_T>& core) {

    if (isabsolute_) {  // Does this measurement refer to an absolute measurement,
      // or is it relative to the last measurement.
      // Get a const ref, so we can read core states.
      const EKFState_T& state = *state_nonconst_new;
      // init variables
      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new;
      Eigen::Matrix<double, nMeasurements, 1> r_old;

      CalculateH(state_nonconst_new, H_new);

      // Get rotation matrices.
      Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
          .conjugate().toRotationMatrix();

      // Construct residuals:
      // Position
      r_old.block<3, 1>(0, 0) = z_p_
          - (state.Get<StateDefinition_T::p>()
              + C_q.transpose() * state.Get<StateDefinition_T::p_ip>());
      
      // Velocity
      r_old.block<3, 1>(3, 0) = z_v_
          - state.Get<StateDefinition_T::v>();


      if (!CheckForNumeric(r_old, "r_old")) {
        MSF_ERROR_STREAM("gps r_old: "<<r_old);
        return;
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(H_new, "H_old")) {
        MSF_ERROR_STREAM("gps H_old: "<<H_new);
        return;
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(R_, "R_")) {
        MSF_ERROR_STREAM("gps R_: "<<R_);
        return;
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }







    msf_core::MSF_Core<EKFState_T>::ErrorStateCov _P;
    Eigen::Matrix<double, msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime, msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> sP;
    sP = 0.5*(_P.transpose()+_P);


      // residual covariance, (inverse)
      Eigen::Matrix<double, nMeasurements, nMeasurements> S_I =
       (H_new * sP * H_new.transpose() + R_).inverse();


      // fault detection (mahalanobis distance !! )
      float beta = (r_old.transpose() * (S_I * r_old))(0, 0);
      // MSF_WARN_STREAM("gb=" << beta);
      if(std::isnan(beta) || std::isinf(beta))
        return;

      // printf("position beta\t%.2f\n", beta);
      // printf("%.3f\t%.3f\t%.3f\n%.3f\n\n\n",
            // _P(3,3),_P(4,4),_P(5,5),beta);

        if (beta > BETA_TABLE[nMeasurements]) {
          if (_gpsFault < FAULT_MINOR) {
            printf("gps FAULT_MINOR\n");
            _gpsFault = FAULT_MINOR;
          }else if(_gpsFault >= FAULT_MINOR && beta > 10000) { //from default mah_threshold
            printf("gps bad\n");
            _gpsFault = FAULT_SEVERE;
          }
        } else if (_gpsFault) {
          _gpsFault = FAULT_NONE;
        }

        if (_gpsFault < fault_lvl_disable) {
          // Call update step in base class.
          this->CalculateAndApplyCorrection(state_nonconst_new, core, H_new, r_old,
                                        R_);
        } else {
          return;
        }







      
    } else {
      MSF_ERROR_STREAM_THROTTLE(
          1, "You chose to apply the position measurement "
          "as a relative quantitiy, which is currently not implemented.");
    }
  }
};
}  // namespace position_measurement
}  // namespace msf_updates

#endif  // POSITION_MEASUREMENT_HPP_
