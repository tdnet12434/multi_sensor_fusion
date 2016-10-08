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
#ifndef AHRS_MEASUREMENT_HPP_
#define AHRS_MEASUREMENT_HPP_ 

#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>
#include <msf_core/eigen_utils.h>
#include <sensor_msgs/Imu.h>

namespace msf_updates {
namespace ahrs_measurement {
enum {
  nMeasurements = 3
};

enum fault_t {
  FAULT_NONE = 0,
  FAULT_MINOR,
  FAULT_SEVERE
};
fault_t _flowFault;

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
    sensor_msgs::Imu,
    Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState> AhrsMeasurementBase;
struct AhrsMeasurement : public AhrsMeasurementBase {
 private:
  typedef AhrsMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

    virtual void MakeFromSensorReadingImpl(measptr_t msg) {

    Eigen::Matrix<double, nMeasurements,
        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();


    z_q_ = Eigen::Quaternion<double> (msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    


    const double s_zq = n_zq_ * n_zq_;
    R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zq, s_zq, s_zq)
        .finished().asDiagonal();

  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Quaternion<double> z_q_;  /// Position measurement.
  double n_zq_;  /// Position measurement noise.
  double dt;
 
  bool fixed_covariance_;
  int fixedstates_;

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~AhrsMeasurement() {
  }
  AhrsMeasurement(double n_zq, bool fixed_covariance,
                      bool isabsoluteMeasurement, int sensorID, int fixedstates)
      : AhrsMeasurementBase(isabsoluteMeasurement, sensorID),
        n_zq_(n_zq),
        fixed_covariance_(fixed_covariance),
        fixedstates_(fixedstates)
         {
  }
  virtual std::string Type() {
    return "ahrs";
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
    //w x y z preset drift 10 deg
    Eigen::Quaternion<double> q_ = state.Get<StateDefinition_T::q>();
    q_.normalize();


    // Get indices of states in error vector.
    enum {
      idxstartcorr_p_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::p>::value,
      idxstartcorr_v_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::v>::value,
      idxstartcorr_q_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::q>::value,
      idxstartcorr_L_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::L>::value
    };
    // double qx=q_.x();
    // double qy=q_.y();
    // double qz=q_.z();
    // double qw=q_.w();
    // Eigen::Matrix<double, 4,3> Qdth;
    // Qdth << -qx, -qy, -qz,
    //          qw, -qz,  qy,
    //          qz,  qw, -qx,
    //         -qy,  qx,  qw;
    // Qdth*=0.5;
    H.block<3, 3>(0, idxstartcorr_q_) =  Eigen::MatrixXd::Identity(3,3);//Qdth;

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




      Eigen::Quaternion<double> q_err;
      q_err = state.Get<StateDefinition_T::q>().conjugate() * z_q_;
      r_old.block<3,1>(0,0) = q_err.vec() / q_err.w() * 2;
      // r_old.block<1,1>(3,0) = q_err.w();
      // else
      //   return;

      //         + C_q.transpose() * state.Get<StateDefinition_T::p_ip>());
      if (!CheckForNumeric(r_old, "r_old")) {
        // Eigen::Matrix<double, 3, 3> ident
        // r_old.block<3, 1>(0, 0) =  ;
        MSF_ERROR_STREAM("ahrs r_old: "<<r_old);
        return;
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(H_new, "H_old")) {
        MSF_ERROR_STREAM("ahrs H_old: "<<H_new);
        return;
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(R_, "R_")) {
        MSF_ERROR_STREAM("ahrs R_: "<<R_);
        return;
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }



          this->CalculateAndApplyCorrection(state_nonconst_new, core, H_new, r_old,
                                        R_);
    } 
};
}  // namespace position_measurement
}  // namespace msf_updates

#endif  // POSITION_MEASUREMENT_HPP_
