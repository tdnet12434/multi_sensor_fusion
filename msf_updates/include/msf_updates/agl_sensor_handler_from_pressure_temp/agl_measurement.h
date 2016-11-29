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
#ifndef AGL_MEASUREMENT_HPP_
#define AGL_MEASUREMENT_HPP_

#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>

namespace agl_measurement {
enum {
  z_tz=0,
  nMeasurements
};

/**
 * \brief A measurement as provided by a agl sensor.
 */
typedef msf_core::MSF_Measurement<sensor_msgs::Range,
    Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState> AglMeasurementBase;
struct AglMeasurement : public AglMeasurementBase {
 private:
  typedef AglMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void MakeFromSensorReadingImpl(measptr_t msg) {
    Eigen::Matrix<double, nMeasurements,
        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();

    // Get measurements.
    z_p_ = Eigen::Matrix<double, 1, 1>::Constant(msg->range);

    const double s_zp = n_zp_ * n_zp_;
    R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zp).finished()
        .asDiagonal();
  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix<double, 1, 1> z_p_;  /// Agl measurement.
  double n_zp_;  /// Agl measurement noise.
. 
bool sonar_healhy;
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~AglMeasurement() { }
  AglMeasurement(double n_zp, bool isabsoluteMeasurement, int sensorID)
      : AglMeasurementBase(isabsoluteMeasurement, sensorID),
        n_zp_(n_zp) { }
  virtual std::string Type() {
    return "agl";
  }
  /**
   * The method called by the msf_core to apply the measurement represented by
   * this object.
   */
  virtual void Apply(shared_ptr<EKFState_T> non_const_state,
                     msf_core::MSF_Core<EKFState_T>& core) {
    // Init variables.
    Eigen::Matrix<double, nMeasurements,
        msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();

    if (non_const_state->time == msf_core::constants::INVALID_TIME) {
      MSF_WARN_STREAM(
          "Apply agl update was called with an invalid state.");
      return;  // Early abort.
    }

    const EKFState_T& state = *non_const_state;

    H.setZero();

    // Get rotation matrices.
    Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
        .toRotationMatrix();


    Eigen::Vector3d euler = C_q.eulerAngles(2, 1, 0);

    double eff = cos(euler[2]) * cos(euler[1]);

    if (z_p_(0) < 0.3 || z_p_(0) > 4.0) {
      sonar_healhy = false; 
    }else {
      sonar_healhy = true;
    }
    Eigen::Matrix<double, 1, 1> z_sonar_compensated;  /// Agl measurement
    z_sonar_compensated = z_p_ * eff;
    printf("%.2f\n",z_sonar_compensated(0) );

    // Get indices of states in error vector.
    enum {
      idxstartcorr_p_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::p>::value,
      idxstartcorr_v_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::v>::value,
      idxstartcorr_q_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::q>::value,
      idxstartcorr_L_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::L>::value,
      idxstartcorr_qif_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::q_if>::value,
      idxstartcorr_tz_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::tz>::value
    };

        printf("gg%d\n",sonar_healhy);
        H.block<1, 1>(z_tz, idxstartcorr_p_+2)(0) = (sonar_healhy ? 1:0);
        H.block<1, 1>(z_tz, idxstartcorr_tz_)(0) = (sonar_healhy ? -1:0);

      

    // Construct residuals.
    // Height.
    r_old.block<1, 1>(z_tz, 0) = z_sonar_compensated 
                                      - (state.Get<StateDefinition_T::p>().block<1, 1>(2, 0)
                                          - state.Get<StateDefinition_T::tz>().block<1, 1>(0, 0));
    // Call update step in base class.
    this->CalculateAndApplyCorrection(non_const_state, core, H_old, r_old, R_);
  }
};
}  // namespace agl_measurement
#endif  // POSE_MEASUREMENT_HPP_
