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
#ifndef POSE_MEASUREMENT_HPP_
#define POSE_MEASUREMENT_HPP_

#include <msf_core/msf_types.h>
#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>
#include <msf_updates/PoseDistorter.h>

namespace msf_updates {
namespace pose_measurement {
enum {
  nMeasurements = 6
};
/**
 * \brief A measurement as provided by a pose tracking algorithm.
 */
typedef msf_core::MSF_Measurement<geometry_msgs::PoseWithCovarianceStamped,
    Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState> PoseMeasurementBase;

template<
    int StateLIdx = EKFState::StateDefinition_T::L,
    int StatePwvIdx = EKFState::StateDefinition_T::p_wv
    >
struct PoseMeasurement : public PoseMeasurementBase {
 private:
  typedef PoseMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void MakeFromSensorReadingImpl(measptr_t msg) {
    Eigen::Matrix<double, nMeasurements,
        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();

    // Get measurements.
    z_p_ = Eigen::Matrix<double, 3, 1>(msg->pose.pose.position.x,
                                       msg->pose.pose.position.y,
                                       msg->pose.pose.position.z);
    z_q_ = Eigen::Quaternion<double>(msg->pose.pose.orientation.w,
                                     msg->pose.pose.orientation.x,
                                     msg->pose.pose.orientation.y,
                                     msg->pose.pose.orientation.z);


    if (fixed_covariance_) {  // Take fix covariance from reconfigure GUI.
      const double s_zp = n_zp_ * n_zp_;
      const double s_zq = n_zq_ * n_zq_;
      R_ =
          (Eigen::Matrix<double, nMeasurements, 1>() << s_zp, s_zp, s_zp, s_zq, s_zq, s_zq)
              .finished().asDiagonal();
    } else {  // Take covariance from sensor.
      R_.block<6, 6>(0, 0) = Eigen::Matrix<double, 6, 6>(
          &msg->pose.covariance[0]);

      if (msg->header.seq % 100 == 0) {  // Only do this check from time to time.
        if (R_.block<6, 6>(0, 0).determinant() < -0.001)
          MSF_WARN_STREAM_THROTTLE(
              60,
              "The covariance matrix you provided for " "the pose sensor is not positive definite: "<<(R_.block<6, 6>(0, 0)));
      }

      // Clear cross-correlations between q and p.
      R_.block<3, 3>(0, 3) = Eigen::Matrix<double, 3, 3>::Zero();
      R_.block<3, 3>(3, 0) = Eigen::Matrix<double, 3, 3>::Zero();
    }
  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Quaternion<double> z_q_;  /// Attitude measurement camera seen from world.
  Eigen::Matrix<double, 3, 1> z_p_;  /// Position measurement camera seen from world.
  double n_zp_, n_zq_;  /// Position and attitude measurement noise.

  bool measurement_world_sensor_;
  bool fixed_covariance_;
  msf_updates::PoseDistorter::Ptr distorter_;
  int fixedstates_;

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  enum AuxState {
    L = StateLIdx,
    p_wv = StatePwvIdx
  };

  virtual ~PoseMeasurement() {
  }
  PoseMeasurement(double n_zp, double n_zq, bool measurement_world_sensor,
                  bool fixed_covariance, bool isabsoluteMeasurement,
                  int sensorID, int fixedstates,
                  msf_updates::PoseDistorter::Ptr distorter =
                      msf_updates::PoseDistorter::Ptr())
      : PoseMeasurementBase(isabsoluteMeasurement, sensorID),
        n_zp_(n_zp),
        n_zq_(n_zq),
        measurement_world_sensor_(measurement_world_sensor),
        fixed_covariance_(fixed_covariance),
        distorter_(distorter),
        fixedstates_(fixedstates) {
  }
  virtual std::string Type() {
    return "pose";
  }

  virtual void CalculateH(
      shared_ptr<EKFState_T> state_in,
      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& H) {
    const EKFState_T& state = *state_in;  // Get a const ref, so we can read core states.

    H.setZero();

    // Get rotation matrices.
    Eigen::Matrix<double, 3, 3> C_wi = state.Get<StateDefinition_T::q>()
        .toRotationMatrix();



    // Get indices of states in error vector.
    enum {
      kIdxstartcorr_p = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::p>::value,
      kIdxstartcorr_v = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::v>::value,
      kIdxstartcorr_q = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::q>::value,

      kIdxstartcorr_L = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateLIdx>::value,
      kIdxstartcorr_pwv = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StatePwvIdx>::value
    };

    // Read the fixed states flags.
    bool scalefix = (fixedstates_ & 1 << StateLIdx);
    bool driftwvposfix = (fixedstates_ & 1 << StatePwvIdx);

    // Set crosscov to zero for fixed states.
    if (scalefix)
      state_in->ClearCrossCov<StateLIdx>();
    if (driftwvposfix)
      state_in->ClearCrossCov<StatePwvIdx>();


    // Construct H matrix.
    // Position:
    H.block<3, 3>(0, kIdxstartcorr_p) = Eigen::Matrix<double, 3, 3>::Identity()*state.Get<StateLIdx>()(0);  // p

    if (scalefix) {
      H.block<3, 1>(0, kIdxstartcorr_L).setZero();
    } else {
      H.block<3, 1>(0, kIdxstartcorr_L) =-state.Get<StatePwvIdx>() + state.Get<StateDefinition_T::p>();  // L
    }


    // TODO (slynen): Check scale commenting
    if (driftwvposfix) {
      H.block<3, 3>(0, kIdxstartcorr_pwv).setZero();
    } else {
      H.block<3, 3>(0, kIdxstartcorr_pwv) = (-Eigen::Matrix<double, 3, 3>::Identity()
           * state.Get<StateLIdx>()(0)).eval();  //p_wv
    }

    // Attitude.
    H.block<3, 3>(3, kIdxstartcorr_q) = Eigen::Matrix<double, 3, 3>::Identity().eval();  // q

    // This line breaks the filter if a position sensor in the global frame is
    // available or if we want to set a global yaw rotation.
    //H.block<1, 1>(6, kIdxstartcorr_qwv + 2) = Eigen::Matrix<double, 1, 1>::
    // Constant(driftwvattfix ? 0.0 : 1.0); // fix vision world yaw drift because unobservable otherwise (see PhD Thesis)

  }

  /**
   * The method called by the msf_core to apply the measurement represented by
   * this object
   */
  virtual void Apply(shared_ptr<EKFState_T> state_nonconst_new,
                     msf_core::MSF_Core<EKFState_T>& core) {

      // Get a const ref, so we can read core states
      const EKFState_T& state = *state_nonconst_new;
      // init variables
      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new;
      Eigen::Matrix<double, nMeasurements, 1> r_old;

      CalculateH(state_nonconst_new, H_new);

      // Get rotation matrices.
      Eigen::Matrix<double, 3, 3> C_wi = state.Get<StateDefinition_T::q>()
          .toRotationMatrix();

      // Construct residuals.
      // Position.
      r_old.block<3, 1>(0, 0) = z_p_
          - (-state.Get<StatePwvIdx>()
                  + state.Get<StateDefinition_T::p>()
                  )
              * state.Get<StateLIdx>();

      // Attitude.
      Eigen::Quaternion<double> q_err;  // q_err = \hat{q}_vc * q_cv
      q_err = (state.Get<StateDefinition_T::q>()).conjugate() * z_q_;
      r_old.block<3, 1>(3, 0) = q_err.vec() / q_err.w() * 2;
      

      if (!CheckForNumeric(r_old, "r_old")) {
        MSF_ERROR_STREAM("pose r_old: "<<r_old);
        return;
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(H_new, "H_old")) {
        MSF_ERROR_STREAM("pose H_old: "<<H_new);
        return;
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(R_, "R_")) {
        MSF_ERROR_STREAM("pose R_: "<<R_);
        return;
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }


      // msf_core::MSF_Core<EKFState_T>::ErrorStateCov &_P = state_nonconst_new->P;
      // _P = 0.5*(_P.transpose()+_P);

      // residual covariance, (inverse)
      Eigen::Matrix<double, nMeasurements, nMeasurements> S_I =
       (H_new * state_nonconst_new->P * H_new.transpose() + R_).inverse();


      // fault detection (mahalanobis distance !! )
      double beta = (r_old.transpose() * (S_I * r_old))(0, 0);
      if(std::isnan(beta) || std::isinf(beta))
        return;
      // printf("vis_d\t\t\t%.2f\n", beta);




      // Call update step in base class.
      this->CalculateAndApplyCorrection(state_nonconst_new, core, H_new, r_old,
                                        R_);
    
  }
};

}  // namespace msf_pose_sensor
}  // namespace msf_updates
#endif  // POSE_MEASUREMENT_HPP_
