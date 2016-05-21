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
  nMeasurements = 3
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
    z_m_ = Eigen::Matrix<double, 3, 1>(msg->point.x, msg->point.y,
                                       msg->point.z);
    z_m_.normalize();

    if (fixed_covariance_)  //  take fix covariance from reconfigure GUI
    {

      const double s_zm = n_zm_ * n_zm_;
      R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zm, s_zm, s_zm)
          .finished().asDiagonal();

    } else {  // Tke covariance from sensor.

      R_.block<3, 3>(0, 0) = msf_core::Matrix3(&msg->covariance[0]);

      if (msg->header.seq % 100 == 0) {  // Only do this check from time to time.
        if (R_.block<3, 3>(0, 0).determinant() < -0.01)
          MSF_WARN_STREAM_THROTTLE(
              60, "The covariance matrix you provided for "
              "the mag sensor is not positive definite");
      }
    }
  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double, 3, 1> z_m_;  /// Mag measurement.
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
          StateDefinition_T::q>::value,
      idxstartcorr_q_mi_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::q_mi_>::value,
      idxstartcorr_alpha_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::alpha_>::value,
      idxstartcorr_beta_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::beta_>::value,
    };
    ////////////////////////////////////////////GPS POSITION//////////////////////////////////////
    // // Preprocess for elements in H matrix.
    // Eigen::Matrix<double, 3, 3> p_prism_imu_sk = Skew(
    //     state.Get<StateDefinition_T::p_ip>());



    // bool fixed_p_pos_imu = (fixedstates_ & 1 << StateDefinition_T::p_ip);

    // // Clear crosscorrelations.
    // if (fixed_p_pos_imu)
    //   state_in->ClearCrossCov<StateDefinition_T::p_ip>();

    // // Construct H matrix:
    // // Mag:
    // H.block<3, 3>(0, idxstartcorr_p_) = Eigen::Matrix<double, 3, 3>::Identity();  // p

    // H.block<3, 3>(0, idxstartcorr_q_) = -C_q.transpose() * p_prism_imu_sk;  // q

    // H.block<3, 3>(0, idxstartcorr_p_pi_) =
    //     fixed_p_pos_imu ?
    //         Eigen::Matrix<double, 3, 3>::Zero() : (C_q.transpose()).eval();  //p_pos_imu_
    ////////////////////////////////////////////////////////////////////////////////////////////////






    // copy from vismaggps 
       C_mi = state.Get<StateDefinition_T::q_mi_>().conjugate().toRotationMatrix();


      // z_m_ = MagBuff_.back().mag_;
       const static float PI =3.14159265359;
      // Eigen::Matrix<double,3,1> magRvec;
      // magRvec  << n_zm_*n_zm_,n_zm_*n_zm_,n_zm_*n_zm_;
      // R.block(nGPSMeas_,nGPSMeas_,nMagMeas_,nMagMeas_) = magRvec.asDiagonal();
  //    customMeas->p_m_ = z_m_;

      double alpha_ = state.Get<StateDefinition_T::alpha_>()(0,0);  //Elevator angle
      double  beta_ = state.Get<StateDefinition_T::beta_>()(0,0);   //Azimuth angle
      
      vec_m << cos(beta_)*cos(alpha_), sin(beta_)*cos(alpha_), sin(alpha_);
      Eigen::Matrix<double,3,1> vec_m_dalpha;
      vec_m_dalpha << -cos(beta_)*sin(alpha_), -sin(beta_)*sin(alpha_), cos(alpha_);
      Eigen::Matrix<double,3,1> vec_m_dbeta;
      vec_m_dbeta << -sin(beta_)*cos(alpha_), cos(beta_)*cos(alpha_), 0;

      Eigen::Matrix<double,3,3> mw_sk1;
      Eigen::Matrix<double,3,1> vec1_mw = C_q*vec_m;
      mw_sk1 << 0, -vec1_mw(2), vec1_mw(1)
          ,vec1_mw(2), 0, -vec1_mw(0)
          ,-vec1_mw(1), vec1_mw(0), 0;

      Eigen::Matrix<double,3,3> mw_sk2;
      Eigen::Matrix<double,3,1> vec2_mw = C_mi*C_q*vec_m;
      mw_sk2 << 0, -vec2_mw(2), vec2_mw(1)
          ,vec2_mw(2), 0, -vec2_mw(0)
          ,-vec2_mw(1), vec2_mw(0), 0;

      // construct H matrix using H-blockx :-)
      H.block<3,3>(0,idxstartcorr_q_)/*(nGPSMeas_,6,3,3)*/ = C_mi*mw_sk1;
      H.block<3,3>(0,idxstartcorr_q_mi_)/*(nGPSMeas_,25,3,3)*/ = mw_sk2; //start at q_mi_
      H.block<3,1>(0,idxstartcorr_alpha_)/*(nGPSMeas_,34,3,1)*/ = C_mi*C_q*vec_m_dalpha;  //start at alpha
      H.block<3,1>(0,idxstartcorr_beta_)/*(nGPSMeas_,35,3,1)*/ =C_mi*C_q*vec_m_dbeta;    //start at beta

            





























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
      r_old.block<3, 1>(0,0)/*(nGPSMeas_,0,3,1)*/ = z_m_ - C_mi*C_q*(vec_m);  

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
