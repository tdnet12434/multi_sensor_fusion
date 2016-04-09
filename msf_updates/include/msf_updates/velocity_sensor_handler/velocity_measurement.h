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
#ifndef VELOCITY_MEASUREMENT_HPP_
#define VELOCITY_MEASUREMENT_HPP_

#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>
#include <msf_core/eigen_utils.h>
#include <sensor_msgs/Imu.h>

namespace msf_updates {
namespace velocity_measurement {
enum {
  nMeasurements = 3
};

/**
 * \brief A measurement as provided by a position sensor, e.g. Total Station, GPS.
 */
typedef msf_core::MSF_Measurement<
    sensor_msgs::Imu,
    Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState> VelocityMeasurementBase;
struct VelocityMeasurement : public VelocityMeasurementBase {
 private:
  typedef VelocityMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void MakeFromSensorReadingImpl(measptr_t msg) {

    Eigen::Matrix<double, nMeasurements,
        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();


    a_bf_ = Eigen::Matrix<double, 3, 1> (msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    
    if (fixed_covariance_)  //  take fix covariance from reconfigure GUI
    {

      const double s_zv = n_zv_ * n_zv_;
      R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zv, s_zv, 10)
          .finished().asDiagonal();

    } else {  // Tke covariance from sensor.

      R_.block<3, 3>(0, 0) = msf_core::Matrix3(&msg->linear_acceleration_covariance[0]);

      if (msg->header.seq % 100 == 0) {  // Only do this check from time to time.
        if (R_.block<3, 3>(0, 0).determinant() < -0.01)
          MSF_WARN_STREAM_THROTTLE(
              60, "The covariance matrix you provided for "
              "the Velocity sensor is not positive definite");
      }
    }
  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double, 3, 1> a_bf_;  /// bf measurement.
  Eigen::Matrix<double, 3, 1> z_v_;  /// Position measurement.
  Eigen::Matrix<double, 3, 3> temp;
  double n_zv_;  /// Position measurement noise.
  double drag_;

  bool fixed_covariance_;
  int fixedstates_;

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~VelocityMeasurement() {
  }
  VelocityMeasurement(double n_zv, bool fixed_covariance,
                      bool isabsoluteMeasurement, int sensorID, int fixedstates, double drag)
      : VelocityMeasurementBase(isabsoluteMeasurement, sensorID),
        n_zv_(n_zv),
        fixed_covariance_(fixed_covariance),
        fixedstates_(fixedstates),
        drag_(drag) {
  }
  virtual std::string Type() {
    return "velocity";
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

    Eigen::Matrix<double, 3, 1>  g;
    g << 0, 0, 9.80655; /// Gravity.
    // Get measurement.
    z_v_ = C_q*(a_bf_ - state.Get<StateDefinition_T::q>().inverse() * g);//Eigen::Matrix<double, 3, 1>(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    // printf("%f %f %f\n",z_v_(0,0),z_v_(1,0),z_v_(2,0));
    // Preprocess for elements in H matrix.
    // Eigen::Matrix<double, 3, 3> p_prism_imu_sk = Skew(
    //     state.Get<StateDefinition_T::p_ip>());

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

    // bool fixed_p_pos_imu = (fixedstates_ & 1 << StateDefinition_T::p_ip);

    // // Clear crosscorrelations.
    // if (fixed_p_pos_imu)
    //   state_in->ClearCrossCov<StateDefinition_T::p_ip>();

    // Construct H matrix:
    const double body_k_ = drag_;
    Eigen::Matrix<double, 3, 3> body_k = (Eigen::Matrix<double, 3, 1>() << -body_k_, -body_k_,0).finished().asDiagonal();
    // velocity:
    H.block<3, 3>(0, idxstartcorr_v_) = C_q * body_k;  // v
    temp=C_q * body_k;
    // printf("%.3f %.3f %.3f\n%.3f %.3f %.3f\n%.3f %.3f %.3f\n\n\n"
    //   ,body_k(0,0),body_k(0,1),body_k(0,2)
    //   ,body_k(1,0),body_k(1,1),body_k(1,2)
    //   ,body_k(2,0),body_k(2,1),body_k(2,2));

    //H.block<3, 3>(1, idxstartcorr_v_+1) = -C_q.transpose() * p_prism_imu_sk;  // q

    // H.block<3, 3>(0, idxstartcorr_p_pi_) =
    //     fixed_p_pos_imu ?
    //         Eigen::Matrix<double, 3, 3>::Zero() : (C_q.transpose()).eval();  //p_pos_imu_

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

      Eigen::Matrix<double, 3, 1>  g;
      g << 0, 0, 9.80655; /// Gravity.
      // Construct residuals:
      // Position
      // r_old.block<3, 1>(0, 0) = z_v_
      //     - (state.Get<StateDefinition_T::p>()
      //         + C_q.transpose() * state.Get<StateDefinition_T::p_ip>());
      // // Velocity
      r_old.block<3, 1>(0, 0) = z_v_
          - temp*(state.Get<StateDefinition_T::v>());
      //         + C_q.transpose() * state.Get<StateDefinition_T::p_ip>());
      if (!CheckForNumeric(r_old, "r_old")) {
        MSF_ERROR_STREAM("r_old: "<<r_old);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(H_new, "H_old")) {
        MSF_ERROR_STREAM("H_old: "<<H_new);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(R_, "R_")) {
        MSF_ERROR_STREAM("R_: "<<R_);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }

      // Call update step in base class.
      this->CalculateAndApplyCorrection(state_nonconst_new, core, H_new, r_old,
                                        R_);
    } else {
      MSF_ERROR_STREAM_THROTTLE(
          1, "You chose to apply the Velocity measurement "
          "as a relative quantitiy, which is currently not implemented.");
    }
  }
};
}  // namespace position_measurement
}  // namespace msf_updates

#endif  // POSITION_MEASUREMENT_HPP_
