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
#include <msf_core/eigen_utils.h>
#include <sensor_msgs/Range.h>

namespace msf_updates {
namespace agl_measurement {
enum {
  z_tz=0,
  nMeasurements
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
static const double BETA_TABLE[7] = {0,
            8.82050518214,
            12.094592431,
            13.9876612368,
            16.0875642296,
            17.8797700658,
            19.6465647819,
           };





/**
 * \brief A measurement as provided by a agl sensor, e.g. Total Station, GPS.
 */
typedef msf_core::MSF_Measurement<
    sensor_msgs::Range,
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

    // Get measurement.
    z_p_ = Eigen::Matrix<double,1,1>::Constant(msg->range);
    // printf("sonar = %.2f\n", z_p_(0));


    const double s_zsonar = n_zp_ * n_zp_;
    R_ (0) = s_zsonar;
    // printf("R_=%.2f\n", R_(0));

  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double, 1, 1> z_p_;  /// Agl measurement.
  double n_zp_;  /// Agl measurement noise.

  bool fixed_covariance_;
  int fixedstates_;

  Eigen::Matrix<double,1,1> z_sonar_compensated;
  bool sonar_healhy;

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~AglMeasurement() {
  }
  AglMeasurement(double n_zp, bool fixed_covariance,
                      bool isabsoluteMeasurement, int sensorID, int fixedstates)
      : AglMeasurementBase(isabsoluteMeasurement, sensorID),
        n_zp_(n_zp),
        fixed_covariance_(fixed_covariance),
        fixedstates_(fixedstates) {
  }
  virtual std::string Type() {
    return "agl";
  }

  virtual void CalculateH(
      shared_ptr<EKFState_T> state_in,
      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& H) {
    const EKFState_T& state = *state_in;  // Get a const ref, so we can read core states.

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

    z_sonar_compensated = z_p_ * eff;
    // printf("%.2f\n",z_sonar_compensated(0) );

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

        // printf("gg%d\n",sonar_healhy);
        H.block<1, 1>(z_tz, idxstartcorr_p_+2)(0) = (sonar_healhy ? 1:0);
        H.block<1, 1>(z_tz, idxstartcorr_tz_)(0) = (sonar_healhy ? -1:0);

      
  }

  /**
   * The method called by the msf_core to apply the measurement represented by this object.
   */
  virtual void Apply(shared_ptr<EKFState_T> state_nonconst_new,
                     msf_core::MSF_Core<EKFState_T>& core) {

   // Get a const ref, so we can read core states.
      const EKFState_T& state = *state_nonconst_new;
      // init variables
      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new;
      Eigen::Matrix<double, nMeasurements, 1> r_old;



      CalculateH(state_nonconst_new, H_new);




      // Get rotation matrices.
      Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
          .toRotationMatrix();

        r_old.block<1, 1>(z_tz, 0) = z_sonar_compensated 
                                      - (state.Get<StateDefinition_T::p>().block<1, 1>(2, 0)
                                          - state.Get<StateDefinition_T::tz>().block<1, 1>(0, 0));
                           


      //         + C_q.transpose() * state.Get<StateDefinition_T::p_ip>());
      if (!CheckForNumeric(r_old, "r_old")) {
        // Eigen::Matrix<double, 3, 3> ident
        // r_old.block<1, 1>(0, 0) =  ;
        MSF_ERROR_STREAM("agl r_old: "<<r_old);
        return;
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(H_new, "H_old")) {
        MSF_ERROR_STREAM("agl H_old: "<<H_new);
        return;
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(R_, "R_")) {
        MSF_ERROR_STREAM("agl R_: "<<R_);
        return;
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }



      // residual covariance, (inverse)
      Eigen::Matrix<double, nMeasurements, nMeasurements> S_I =
       (H_new * state_nonconst_new->P * H_new.transpose() + R_).inverse();


      // fault detection (mahalanobis distance !! )
      double beta = (r_old.transpose() * (S_I * r_old))(0, 0);
      if(std::isnan(beta) || std::isinf(beta))
        return;
      // printf("agl inno = %.4f\n", beta);


      if(beta < 10) 
          this->CalculateAndApplyCorrection(state_nonconst_new, core, H_new, r_old,
                                        R_);

  }
};
}  // namespace agl_measurement
}  // namespace msf_updates

#endif  // AGL_MEASUREMENT_HPP_
