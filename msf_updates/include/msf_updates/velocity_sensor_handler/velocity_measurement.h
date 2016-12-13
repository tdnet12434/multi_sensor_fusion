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
#include <mavros_msgs/OpticalFlowRad.h>
#include <mavros_msgs/Altitude.h>

namespace msf_updates {
namespace velocity_measurement {
enum {
  z_vx = 0,
  z_vy,
  nMeasurements
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
static const double BETA_TABLE[7] = {0,
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
    mavros_msgs::OpticalFlowRad,
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


    flow = Eigen::Matrix<double, 3, 1> (msg->integrated_x, 
                                        msg->integrated_y,
                                        0);

    dt = msg->integration_time_us*0.000001;

    flow_q = msg->quality;
    Eigen::Matrix<double, 3, 1> gyro_raw(msg->integrated_xgyro, 
                                         msg->integrated_ygyro, 
                                         0);

    /*High pass filter*/
    // static double b = 2*3.1415926*0.1*0.01;
    // static double a = 1/(1+b);
    static Eigen::Matrix<double, 3, 1> gyro_raw_old = gyro_raw;
    static Eigen::Matrix<double, 3, 1> gyro_f = gyro_raw;

    gyro_f = 0.99375604671 * (gyro_f + gyro_raw - gyro_raw_old);
    gyro_raw_old = gyro_raw;
    gyro = gyro_f;

    // printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", flow(0), flow(1), flow(2), gyro_raw(0), gyro(0));

    if (fixed_covariance_)  //  take fix covariance from reconfigure GUI
    {
      //test drive covariance with quality

      const double s_zv = n_zv_ * n_zv_/** 255 / (flow_q != 0 ? flow_q : 1)*/;

      MSF_WARN_STREAM_THROTTLE(1, "flow cov " << s_zv);
      R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zv, s_zv)
          .finished().asDiagonal();

    } else {  // Tke covariance from sensor.

      // R_.block<3, 3>(0, 0) = msf_core::Matrix3(0.01)/*msf_core::Matrix3(&msg->linear_acceleration_covariance[0])*/;

      // if (msg->header.seq % 100 == 0) {  // Only do this check from time to time.
      //   if (R_.block<3, 3>(0, 0).determinant() < -0.01)
      //     MSF_WARN_STREAM_THROTTLE(
      //         60, "The covariance matrix you provided for "
      //         "the Velocity sensor is not positive definite");
      // }
    }
  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double, 3, 1> flow;  /// raw flow measurement.
  Eigen::Matrix<double, 3, 1> z_p_;  /// Position measurement.
  Eigen::Matrix<double, 3, 1> gyro;  /// Gyro filtered
  

  double n_zv_;  /// Position measurement noise.
  bool fixed_covariance_;
  int fixedstates_;
  double flow_minQ_;
  Eigen::Quaternion<double> qif_;
  
  double flow_q;
  double agl_ef;
  double dt;
  bool flow_healhy;
  bool sonar_healhy;
  double res_out;
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~VelocityMeasurement() {
  }
  VelocityMeasurement(double n_zv,
                      bool fixed_covariance,
                      bool isabsoluteMeasurement, 
                      int sensorID, 
                      int fixedstates, 
                      double flow_minQ, 
                      Eigen::Quaternion<double> qif)
      : VelocityMeasurementBase(isabsoluteMeasurement, sensorID),
        n_zv_(n_zv),
        fixed_covariance_(fixed_covariance),
        fixedstates_(fixedstates),
        flow_minQ_(flow_minQ),
        qif_(qif)
         {
  }
  virtual std::string Type() {
    return "velocity";
  }

  virtual void CalculateH(
      shared_ptr<EKFState_T> state_in,
      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& H) {

    if (dt > 0.5f || dt < 1.0e-6f) {
      flow_healhy = false;
    }

    //select absolute or relative measurement
    if(isabsolute_) MSF_WARN_STREAM_ONCE("FLOW: treat as absolute");
    else            MSF_WARN_STREAM_ONCE("FLOW: treat as relative");

    const EKFState_T& state = *state_in;  // Get a const ref, so we can read core states.

    H.setZero();

    // Get rotation matrices.
    Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
        .toRotationMatrix();
    // Get shift matrices
    Eigen::Matrix<double, 3, 3> C_f = state.Get<StateDefinition_T::q_if>()
        .toRotationMatrix();
    //w x y z preset drift 10 deg
    Eigen::Quaternion<double> q_b_i = qif_;
    q_b_i.normalize();
    Eigen::Matrix<double, 3, 3> Shift_q = q_b_i.toRotationMatrix();
      

    Eigen::Vector3d euler = C_q.eulerAngles(2, 1, 0);

    double eff = cos(euler[2]) * cos(euler[1]);


    agl_ef = (state.Get<StateDefinition_T::p>()(2)-state.Get<StateDefinition_T::tz>()(0))* eff;

    if (agl_ef < 0.2 || agl_ef > 4.0) {
      sonar_healhy = false;
    }else {
      sonar_healhy = true;
    }

    printf("flow_go\n");


    // agl_ef = agl * eff;
    // printf("our=%.2f , but=%.2f\n", (state.Get<StateDefinition_T::p>()(2)-state.Get<StateDefinition_T::tz>()(0)), agl);

    if(flow_q < flow_minQ_ || agl_ef <= 0.3)          {flow_healhy = false;}


    Eigen::Matrix<double, 3, 1> delta_b(
                              -(flow(0) - gyro(0)) * agl_ef,
                              -(flow(1) - gyro(1)) * agl_ef,
                               0);
    Eigen::Matrix<double, 3, 1> delta_n = Shift_q*delta_b;

    static Eigen::Matrix<double, 3, 1> flow_n;/// sum flow
    if(isabsolute_) {
      //treat flow as velocity
      flow_n = delta_n/(dt > 1e-6 ? dt : 0.1); //10 hz
    }else{
      //treat flow as position odometry (relative measurement)
      flow_n += delta_n*0.5; //10 hz
    }
        z_p_(0) = flow_n(0);
        z_p_(1) = flow_n(1);
        z_p_(2) = 0;



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

    Eigen::Matrix<double, 3, 3> ident = (Eigen::Matrix<double, 3, 1>() << 1, 1, 0).finished().asDiagonal();
    // velocity:
    Eigen::Matrix<double, 3, 3> skew_v = Skew(state.Get<StateDefinition_T::v>());
    Eigen::Matrix<double, 3, 1> Rtv = C_q.transpose()*state.Get<StateDefinition_T::v>();
    Eigen::Matrix<double, 3, 3> skew_Rtv = Skew(Rtv);

    Eigen::Matrix<double, 2, 3> S;
    S << 1,0,0,
         0,1,0;

    if(isabsolute_)
      {
        H.block<2, 3>(z_vx, idxstartcorr_v_) =  S*C_f.transpose()*C_q.transpose();
        H.block<2, 3>(z_vx, idxstartcorr_q_) =  S*C_f.transpose()*C_q.transpose()*skew_v;
        H.block<2, 3>(z_vx, idxstartcorr_qif_) =  S*C_f.transpose()*skew_Rtv;
      } 
    else
      {
        H.block<2, 3>(0, idxstartcorr_p_) = S*ident;
      }
      



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












  // // polynomial noise model, found using least squares fit
  // // h, h**2, v, v*h, v*h**2
  // const float p[5] = {0.04005232f, -0.00656446f, -0.26265873f,  0.13686658f, -0.00397357f};

  // // prevent extrapolation past end of polynomial fit by bounding independent variables
  // float h = agl_ef;
  // float v = z_p_.norm();
  // const float h_min = 2.0f;
  // const float h_max = 8.0f;
  // const float v_min = 0.5f;
  // const float v_max = 1.0f;

  // if (h > h_max) {
  //   h = h_max;
  // }

  // if (h < h_min) {
  //   h = h_min;
  // }

  // if (v > v_max) {
  //   v = v_max;
  // }

  // if (v < v_min) {
  //   v = v_min;
  // }

  // // compute polynomial value
  // float flow_vxy_stddev = p[0] * h + p[1] * h * h + p[2] * v + p[3] * v * h + p[4] * v * h * h;

  // const double s_zv = flow_vxy_stddev * flow_vxy_stddev;
  // R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zv, s_zv, 999)
  //     .finished().asDiagonal();




      // Get rotation matrices.
      Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
          .toRotationMatrix();

      // Get rotation matrices.
      Eigen::Matrix<double, 3, 3> C_f = state.Get<StateDefinition_T::q_if>()
          .toRotationMatrix();
      //C_q.transpose() is turn body to earth frame 


      // Eigen::Matrix<double, 3, 1>  g;
      // g << 0, 0, 9.80655; /// Gravity.
      // Construct residuals:
      // Position
      // r_old.block<3, 1>(0, 0) = z_p_
      //     - (state.Get<StateDefinition_T::p>()
      //         + C_q.transpose() * state.Get<StateDefinition_T::p_ip>());


      // static int fault_score=15;
      // if(flow_q < flow_minQ_ || agl_ef <= 0.3) {
      //   fault_score ++;
      //   if(fault_score>=15) {
      //     flow_healhy = false;
      //     fault_score=15;
      //   }
      // }else{
      //   fault_score --;
      //   if(fault_score <=0) {
      //     flow_healhy = true;
      //     fault_score = 0;
      //   }
      // }


      // // Velocity
      // if(flow_healhy)
        r_old.block<2, 1>(z_vx, 0) = z_p_.block<2,1>(0,0)
                                      - (C_f.transpose()*C_q.transpose()*state.Get<StateDefinition_T::v>()).block<2, 1>(0, 0);

        // Eigen::Matrix<double,3,1> state_v = state.Get<StateDefinition_T::v>().block<3, 1>(0, 0);
        // MSF_INFO_STREAM(z_p_ << "----" << state_v);
      // else
      //   return;

      //         + C_q.transpose() * state.Get<StateDefinition_T::p_ip>());
      if (!CheckForNumeric(r_old, "r_old")) {
        // Eigen::Matrix<double, 3, 3> ident
        // r_old.block<3, 1>(0, 0) =  ;
        MSF_ERROR_STREAM("flow r_old: "<<r_old);
        return;
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(H_new, "H_old")) {
        MSF_ERROR_STREAM("flow H_old: "<<H_new);
        return;
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(R_, "R_")) {
        MSF_ERROR_STREAM("flow R_: "<<R_);
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
      if(std::isnan(beta) || std::isinf(beta) || beta<0)
        return;
      res_out = beta;
      // printf("%.3f\t%.3f\t%.3f\n%.3f\n\n\n",
            // _P(3,3),_P(4,4),_P(5,5),beta);
      int n_y_flow = 2;

        if (beta > BETA_TABLE[n_y_flow]) {
          if(_flowFault >= FAULT_MINOR && beta > BETA_TABLE[n_y_flow]) { //from default mah_threshold
            printf("flow bad\n");
            _flowFault = FAULT_SEVERE;
          }else{
            _flowFault = FAULT_MINOR;
          }
          if (_flowFault < FAULT_MINOR) {
            printf("flow FAULT_MINOR\n");
            _flowFault = FAULT_MINOR;
          }
        }else{
          _flowFault = FAULT_NONE;
        }

        
        // else if (_flowFault) {
        //   _flowFault = FAULT_NONE;
        // }
        static uint8_t good_count = 0;
        const uint8_t MAX_COUNT = 20;
        const uint8_t CONFIDENT = 15;
        if (_flowFault < fault_lvl_disable) {
          (good_count > MAX_COUNT ? MAX_COUNT : ++good_count);
        } else {
          (good_count < 1 ? 0 : --good_count);
        }
        static bool let_correction=false;
        if(flow_healhy) {  //if above 30cm and quality good check the good_count
          let_correction = (good_count >= CONFIDENT ? true : false);
        }

        // printf("flow_d\t\t%.2f\n", beta);
        if(let_correction) {
          // Call update step in base class.
          this->CalculateAndApplyCorrection(state_nonconst_new, core, H_new, r_old,
                                        R_);
        }else{
          printf("bad flow\n");
          return;
        }
    } 

















    else {
      // MSF_ERROR_STREAM_THROTTLE(
      //     1, "You chose to apply the Velocity measurement "
      //     "as a relative quantitiy, which is currently not implemented.");


       // Init variables: Get previous measurement.
      shared_ptr < msf_core::MSF_MeasurementBase<EKFState_T> > prevmeas_base =
          core.GetPreviousMeasurement(this->time, this->sensorID_);

      if (prevmeas_base->time == msf_core::constants::INVALID_TIME) {
        MSF_WARN_STREAM(
            "The previous measurement is invalid. Could not apply measurement! " "time:"<<this->time<<" sensorID: "<<this->sensorID_);
        return;
      }

      // Make this a pose measurement.
      shared_ptr<VelocityMeasurement> prevmeas = dynamic_pointer_cast
          < VelocityMeasurement > (prevmeas_base);
      if (!prevmeas) {
        MSF_WARN_STREAM(
            "The dynamic cast of the previous measurement has failed. "
            "Could not apply measurement");
        return;
      }

      // Get state at previous measurement.
      shared_ptr<EKFState_T> state_nonconst_old = core.GetClosestState(
          prevmeas->time);

      if (state_nonconst_old->time == msf_core::constants::INVALID_TIME) {
        MSF_WARN_STREAM(
            "The state at the previous measurement is invalid. Could "
            "not apply measurement");
        return;
      }

      // Get a const ref, so we can read core states.
      const EKFState_T& state_new = *state_nonconst_new;
      const EKFState_T& state_old = *state_nonconst_old;

      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new,
          H_old;
      Eigen::Matrix<double, nMeasurements, 1> r_new, r_old;

      CalculateH(state_nonconst_old, H_old);

      H_old *= -1;

      CalculateH(state_nonconst_new, H_new);

      //TODO (slynen): check that both measurements have the same states fixed!
      // Eigen::Matrix<double, 3, 3> C_wv_old, C_wv_new;
      Eigen::Matrix<double, 3, 3> C_q_old, C_q_new;

      // C_wv_new = state_new.Get<StateQwvIdx>().conjugate().toRotationMatrix();
      C_q_new = state_new.Get<StateDefinition_T::q>().conjugate()
          .toRotationMatrix();

      // C_wv_old = state_old.Get<StateQwvIdx>().conjugate().toRotationMatrix();
      C_q_old = state_old.Get<StateDefinition_T::q>().conjugate()
          .toRotationMatrix();

      // Construct residuals.
      // Position:
      Eigen::Matrix<double, 3, 1> diffprobpos =  state_new.Get<StateDefinition_T::p>()
                                               - state_old.Get<StateDefinition_T::p>();


      Eigen::Matrix<double, 3, 1> diffmeaspos = z_p_ - prevmeas->z_p_;

      r_new.block<3, 1>(0, 0) = diffmeaspos - diffprobpos;
      // printf("dpp = %.3f\tdmp = %.3f\n", diffprobpos(0), diffmeaspos(0));

      if (!CheckForNumeric(r_old, "r_old")) {
        MSF_ERROR_STREAM("r_old: "<<r_old);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state_new). ToEigenVector().transpose());
        return;
      }
      // printf("chkr-");
      if (!CheckForNumeric(H_new, "H_old")) {
        MSF_ERROR_STREAM("H_old: "<<H_new);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state_new). ToEigenVector().transpose());
        return;
      }
      // printf("chkH-");
      if (!CheckForNumeric(R_, "R_")) {
        MSF_ERROR_STREAM("R_: "<<R_);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state_new). ToEigenVector().transpose());
        return;
      }
      // printf("chkR-");



      // msf_core::MSF_Core<EKFState_T>::ErrorStateCov _P;



      // // residual covariance, (inverse)
      // Eigen::Matrix<double, 3, 3> S_I =
      //  (H_new * _P * H_new.transpose() + R_).inverse();


      // // fault detection (mahalanobis distance !! )
      // float beta = (r_new.transpose() * (S_I * r_new))(0, 0);
      // printf("calB\n");
      // if(std::isnan(beta) || std::isinf(beta))
      //   return;
      // printf("beta\t%.2f\n", beta);
      // // printf("%.3f\t%.3f\t%.3f\n%.3f\n\n\n",
      //       // _P(3,3),_P(4,4),_P(5,5),beta);
      // int n_y_flow = 2;

      //   if (beta > BETA_TABLE[n_y_flow]) {
      //     if (_flowFault < FAULT_MINOR) {
      //       printf("flow FAULT_MINOR relative\n");
      //       _flowFault = FAULT_MINOR;
      //     }else if(_flowFault >= FAULT_MINOR && beta > 10000) { //from default mah_threshold
      //       printf("flow bad relative\n");
      //       _flowFault = FAULT_SEVERE;
      //     }
      //   } else if (_flowFault) {
      //     _flowFault = FAULT_NONE;
      //   }

      //   if (_flowFault < fault_lvl_disable) {
        
      //     flow_healhy = true;

      //   } else {
      //     flow_healhy = false;
      //     // reset flow integral to current estimate of position
      //     // if a fault occurred
      //     // _flowX = _x(X_x);
      //     // _flowY = _x(X_y);
      //   }


      // if(flow_healhy) {
        // Call update step in base class.
        this->CalculateAndApplyCorrectionRelative(state_nonconst_old,
                                                  state_nonconst_new, core, H_old,
                                                  H_new, r_new, R_);
      // } else {
      //   printf("bad flow relative\n");
      //   return;
      // }
    }
  }
};
}  // namespace position_measurement
}  // namespace msf_updates

#endif  // POSITION_MEASUREMENT_HPP_
