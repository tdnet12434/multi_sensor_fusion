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
#ifndef MAG_MEASUREMENT_HPP_
#define MAG_MEASUREMENT_HPP_

#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>
#include <msf_core/eigen_utils.h>
#include <sensor_fusion_comm/PointWithCovarianceStamped.h>

namespace msf_updates {
namespace mag_measurement {
enum {
  nMeasurements = 1
};

/**
 * \brief A measurement as provided by a mag sensor, e.g. Total Station, GPS.
 */
typedef msf_core::MSF_Measurement<
    sensor_fusion_comm::PointWithCovarianceStamped,
    Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState> MagMeasurementBase;
struct MagMeasurement : public MagMeasurementBase {
 private:
  typedef MagMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void MakeFromSensorReadingImpl(measptr_t msg) {

    Eigen::Matrix<double, nMeasurements,
        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();

    // Get measurement.
    vec_m = Eigen::Matrix<double, 3, 1>(msg->point.x, msg->point.y,
                                       msg->point.z);
    vec_m.normalize();

    double heading = atan2(-vec_m(1),vec_m(0);
    if(heading > M_PI) heading-=2*M_PI;
    else if (heading <-M_PI) heading+=2*M_PI;
    printf("heading = %.2f", heading);

    Eigen::Quaterniond yawq(cos(heading / 2), 0, 0, sin(heading / 2));
    yawq.normalize();
    z_q_ = yawq;
    // if (fixed_covariance_)  //  take fix covariance from reconfigure GUI
    // {

      const double s_zm = n_zm_ * n_zm_;
      R_ = s_zm;

    // } else {  // Tke covariance from sensor.

    //   R_.block<3, 3>(0, 0) = msf_core::Matrix3(&msg->covariance[0]);

    //   if (msg->header.seq % 100 == 0) {  // Only do this check from time to time.
    //     if (R_.block<3, 3>(0, 0).determinant() < -0.01)
    //       MSF_WARN_STREAM_THROTTLE(
    //           60, "The covariance matrix you provided for "
    //           "the mag sensor is not positive definite");
    //   }
    // }
  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Quaternion<double> z_q_;  /// Attitude measurement camera seen from world.
  Eigen::Matrix<double, 3, 1> vec_m;  // x-axis points north (i.e. beta==0)
  double n_zm_;  /// Mag measurement noise.
  Eigen::Matrix<double,3,3> C_mi;

  bool fixed_covariance_;
  int fixedstates_;

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~MagMeasurement() {
  }
  MagMeasurement(double n_zm, bool fixed_covariance,
                      bool isabsoluteMeasurement, int sensorID, int fixedstates)
      : MagMeasurementBase(isabsoluteMeasurement, sensorID),
        n_zm_(n_zm),
        fixed_covariance_(fixed_covariance),
        fixedstates_(fixedstates) {
  }
  virtual std::string Type() {
    return "mag";
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



    // Get indices of states in error vector.
    enum {
      idxstartcorr_p_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::p>::value,
      idxstartcorr_v_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::v>::value,
      idxstartcorr_q_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::q>::value
    };





    // Eigen::Matrix<double, 3, 3> H_q_yaw = (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished().asDiagonal();
    Eigen::Quaternion
    H.block<3, 3>(0, kIdxstartcorr_q) = Eigen::Matrix<double, 3, 3>Identity();  // q

    




















  }

  /**
   * The method called by the msf_core to apply the measurement represented by this object.
   */
  virtual void Apply(shared_ptr<EKFState_T> state_nonconst_new,
                     msf_core::MSF_Core<EKFState_T>& core) {

    if (1/*isabsolute_*/) {  // Does this measurement refer to an absolute measurement,
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
      // Mag
      r_old.block<3, 1>(0,0)/*(nGPSMeas_,0,3,1)*/ = z_q_ - C_mi*C_q*(vec_m);  

      // r_old.block<3, 1>(0, 0) = z_p_
      //     - (state.Get<StateDefinition_T::p>()
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
    }/* else {
      MSF_ERROR_STREAM_THROTTLE(
          1, "You chose to apply the mag measurement "
          "as a relative quantitiy, which is currently not implemented.");
    }*/
  }
};
}  // namespace mag_measurement
}  // namespace msf_updates

#endif  // MAG_MEASUREMENT_HPP_
