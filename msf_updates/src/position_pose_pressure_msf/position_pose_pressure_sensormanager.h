/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
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
#ifndef POSITION_POSE_PRESSURE_SENSOR_MANAGER_H
#define POSITION_POSE_PRESSURE_SENSOR_MANAGER_H

#include <ros/ros.h>

#include <msf_core/msf_core.h>
#include <msf_core/msf_sensormanagerROS.h>
#include <msf_core/msf_IMUHandler_ROS.h>
#include "msf_statedef.hpp"
#include <msf_updates/pose_sensor_handler/pose_sensorhandler.h>
#include <msf_updates/pose_sensor_handler/pose_measurement.h>
#include <msf_updates/position_sensor_handler/position_sensorhandler.h>
#include <msf_updates/position_sensor_handler/position_measurement.h>
 #include <msf_updates/pressure_sensor_handler/pressure_sensorhandler.h>
#include <msf_updates/PositionPosePressureSensorConfig.h>

 #include <msf_updates/velocity_sensor_handler/velocity_sensorhandler.h>
#include <msf_updates/velocity_sensor_handler/velocity_measurement.h>

 namespace msf_updates {

  typedef msf_updates::PositionPosePressureSensorConfig Config_T;
  typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
  typedef shared_ptr<ReconfigureServer> ReconfigureServerPtr;

  class PositionPosePressureSensorManager : public msf_core::MSF_SensorManagerROS<
  msf_updates::EKFState> {
    typedef PositionPosePressureSensorManager this_T;

    typedef msf_pose_sensor::PoseSensorHandler<
    msf_updates::pose_measurement::PoseMeasurement<>, this_T> PoseSensorHandler_T;


    friend class msf_pose_sensor::PoseSensorHandler<
    msf_updates::pose_measurement::PoseMeasurement<>, this_T>;


    typedef msf_position_sensor::PositionSensorHandler<
    msf_updates::position_measurement::PositionMeasurement, this_T> PositionSensorHandler_T;


    friend class msf_position_sensor::PositionSensorHandler<
    msf_updates::position_measurement::PositionMeasurement, this_T>;

    typedef msf_velocity_sensor::VelocitySensorHandler<
    msf_updates::velocity_measurement::VelocityMeasurement, this_T> VelocitySensorHandler_T;
    friend class msf_velocity_sensor::VelocitySensorHandler<
    msf_updates::velocity_measurement::VelocityMeasurement, this_T>;


    typedef msf_pose_sensor::PoseSensorHandler<msf_updates::pose_measurement::PoseMeasurement<
    msf_updates::EKFState::StateDefinition_T::L_2,
    msf_updates::EKFState::StateDefinition_T::q_ic_2,
    msf_updates::EKFState::StateDefinition_T::p_ic_2,
    msf_updates::EKFState::StateDefinition_T::q_wv_2,
    msf_updates::EKFState::StateDefinition_T::p_wv_2
    >,this_T> PoseSensor_2_Handler_T;

  public:
    typedef msf_updates::EKFState EKFState_T;
    typedef EKFState_T::StateSequence_T StateSequence_T;
    typedef EKFState_T::StateDefinition_T StateDefinition_T;

    PositionPosePressureSensorManager(
      ros::NodeHandle pnh = ros::NodeHandle("~/position_pose_pressure_sensor")) {
      imu_handler_.reset(
        new msf_core::IMUHandler_ROS<msf_updates::EKFState>(*this, "msf_core",
          "imu_handler"));

    bool distortmeas = false;  ///< Distort the pose measurements

    pose_handler_.reset(
      new PoseSensorHandler_T(*this, "", "pose_sensor", distortmeas));
    AddHandler(pose_handler_);

    position_handler_.reset(
      new PositionSensorHandler_T(*this, "", "position_sensor"));
    AddHandler(position_handler_);

    pressure_handler_.reset(
        new msf_pressure_sensor::PressureSensorHandler(*this, "","pressure_sensor"));
    AddHandler(pressure_handler_);

    pose_2_handler_.reset(
      new PoseSensor_2_Handler_T(*this, "", "pose2_sensor", distortmeas));
    AddHandler(pose_2_handler_);
    
    velocity_handler_.reset(
      new VelocitySensorHandler_T(*this, "", "velocity_sensor"));
    AddHandler(velocity_handler_);

    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f = boost::bind(&this_T::Config, this, _1,_2);
    reconf_server_->setCallback(f);
  }
  virtual ~PositionPosePressureSensorManager() {
  }

  virtual const Config_T& Getcfg() {
    return config_;
  }

private:
  shared_ptr<msf_core::IMUHandler_ROS<msf_updates::EKFState> > imu_handler_;
  shared_ptr<PoseSensorHandler_T> pose_handler_;
  shared_ptr<PositionSensorHandler_T> position_handler_;
  shared_ptr<msf_pressure_sensor::PressureSensorHandler> pressure_handler_;

  shared_ptr<PoseSensor_2_Handler_T> pose_2_handler_;

  shared_ptr<VelocitySensorHandler_T> velocity_handler_;


  Config_T config_;
  ReconfigureServerPtr reconf_server_;  ///< Dynamic reconfigure server.

  /**
   * \brief Dynamic reconfigure callback.
   */
   virtual void Config(Config_T &config, uint32_t level) {
    config_ = config;
    pose_handler_->SetNoises(config.pose_noise_meas_p,
     config.pose_noise_meas_q);
    pose_handler_->SetDelay(config.pose_delay);

    position_handler_->SetNoises(config.position_noise_meas);
    position_handler_->SetDelay(config.position_delay);

    pressure_handler_->SetNoises(config.press_noise_meas_p);

    pose_2_handler_->SetNoises(config.pose_noise_meas_p_2,
     config.pose_noise_meas_q_2);
    pose_2_handler_->SetDelay(config.pose_delay_2);

    velocity_handler_->SetNoises(config.velocity_flowNoiseXY);
    velocity_handler_->SetDelay(config.velocity_flowDelay);
    velocity_handler_->SetMinQ(config.velocity_flowMinQ);

    if((level & msf_updates::PositionPosePressureSensor_SET_LATLON) && config.reset_coordinate == true)
    {
      position_handler_->SetLatlon(true);
      MSF_WARN_STREAM(
        "---------------------reset LATLON----------------------------------");
      config.reset_coordinate = false;
    }

    
    if ((level & msf_updates::PositionPosePressureSensor_INIT_FILTER)
      && config.core_init_filter == true) {
      Init(config.pose_initial_scale);
    config.core_init_filter = false;
  }

    // Init call with "set height" checkbox.
  if ((level & msf_updates::PositionPosePressureSensor_SET_HEIGHT)
    && config.core_set_height == true) {
    Eigen::Matrix<double, 3, 1> p = pose_handler_->GetPositionMeasurement();
  if (p.norm() == 0) {
    MSF_WARN_STREAM(
      "No measurements received yet to initialize position. Height init "
      "not allowed.");
    return;
  }
  double scale = p[2] / config.core_height;
  Init(scale);
  config.core_set_height = false;
  
}

}

void Init(double scale) const {
  Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m, p_ic, p_vc, p_wv,
  p_ip, p_pos ,p_zero , p_ic_2, p_vc_2, p_wv_2, p_ip_2, flow_pos;
  Eigen::Quaternion<double> q, q_wv, q_ic, q_vc,   q_wv_2, q_ic_2, q_vc_2;
  msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;
  Eigen::Matrix<double, 1, 1> b_p;
    // init values
    g << 0, 0, 9.80655;	/// Gravity.
    b_w << 0, 0, 0;		/// Bias gyroscopes.
    b_a << 0, 0, 0;		/// Bias accelerometer.

    v << 0, 0, 0;			/// Robot velocity (IMU centered).
    w_m << 0, 0, 0;		/// Initial angular velocity.

    q_wv.setIdentity();  // World-vision rotation drift.
    p_wv.setZero();      // World-vision position drift.

    q_wv_2.setIdentity();  // World-vision rotation drift.
    p_wv_2.setZero();      // World-vision position drift.

    P.setZero();  // Error state covariance; if zero, a default initialization in msf_core is used.
    p_zero.setZero();
    p_pos = position_handler_->GetPositionMeasurement();

    p_vc = pose_handler_->GetPositionMeasurement();
    q_vc = pose_handler_->GetAttitudeMeasurement();

    flow_pos = velocity_handler_->GetVelocityMeasurement();

    b_p << pose_handler_->GetPositionMeasurement()(2) / scale
            - pressure_handler_->GetPressureMeasurement()(0);  /// Pressure drift state


    p_vc_2 = pose_2_handler_->GetPositionMeasurement();
    q_vc_2 = pose_2_handler_->GetAttitudeMeasurement();


    MSF_INFO_STREAM(
      "initial measurement vision: pos:["<<p_vc.transpose()<<"] orientation: " <<STREAMQUAT(q_vc));
    MSF_INFO_STREAM(
      "initial measurement position: pos:["<<p_pos.transpose()<<"]");
    MSF_INFO_STREAM(
      "initial measurement position flow init:["<<flow_pos.transpose()<<"]");

    // Check if we have already input from the measurement sensor.
    if (!pose_handler_->ReceivedFirstMeasurement())
      MSF_WARN_STREAM(
        "No measurements received yet to initialize vision position and attitude - "
        "using [0 0 0] and [1 0 0 0] respectively");
    if (!position_handler_->ReceivedFirstMeasurement())
      MSF_WARN_STREAM(
        "No measurements received yet to initialize absolute position - using [0 0 0]");
    if (!velocity_handler_->ReceivedFirstMeasurement())
      MSF_WARN_STREAM(
        "No measurements received yet to initialize flow position - using [0 0 0]");

    ros::NodeHandle pnh("~");
    pnh.param("pose_sensor/init/p_ic/x", p_ic[0], 0.0);
    pnh.param("pose_sensor/init/p_ic/y", p_ic[1], 0.0);
    pnh.param("pose_sensor/init/p_ic/z", p_ic[2], 0.0);

    pnh.param("pose_sensor/init/q_ic/w", q_ic.w(), 1.0);
    pnh.param("pose_sensor/init/q_ic/x", q_ic.x(), 0.0);
    pnh.param("pose_sensor/init/q_ic/y", q_ic.y(), 0.0);
    pnh.param("pose_sensor/init/q_ic/z", q_ic.z(), 0.0);
    q_ic.normalize();

    MSF_INFO_STREAM("p_ic: " << p_ic.transpose());
    MSF_INFO_STREAM("q_ic: " << STREAMQUAT(q_ic));

    //pose2-----------------------------------------

    pnh.param("pose2_sensor/init/p_ic/x", p_ic_2[0], 0.0);
    pnh.param("pose2_sensor/init/p_ic/y", p_ic_2[1], 0.0);
    pnh.param("pose2_sensor/init/p_ic/z", p_ic_2[2], 0.0);

    pnh.param("pose2_sensor/init/q_ic/w", q_ic_2.w(), 1.0);
    pnh.param("pose2_sensor/init/q_ic/x", q_ic_2.x(), 0.0);
    pnh.param("pose2_sensor/init/q_ic/y", q_ic_2.y(), 0.0);
    pnh.param("pose2_sensor/init/q_ic/z", q_ic_2.z(), 0.0);
    q_ic_2.normalize();

    MSF_INFO_STREAM("p_ic_2: " << p_ic_2.transpose());
    MSF_INFO_STREAM("q_ic_2: " << STREAMQUAT(q_ic_2));




    pnh.param("position_sensor/init/p_ip/x", p_ip[0], 0.0);
    pnh.param("position_sensor/init/p_ip/y", p_ip[1], 0.0);
    pnh.param("position_sensor/init/p_ip/z", p_ip[2], 0.0);

    // Calculate initial attitude and position based on sensor measurements
    // here we take the attitude from the pose sensor and augment it with
    // global yaw init.
    /*double yawinit = config_.position_yaw_init / 180 * M_PI;
    Eigen::Quaterniond yawq(cos(yawinit / 2), 0, 0, sin(yawinit / 2));
    yawq.normalize();*/

    // q = yawq;
    q = q_vc_2;
    //q = q_vc;
    // q_wv = (q * q_ic * q_vc.conjugate()).conjugate();

    MSF_WARN_STREAM("q " << STREAMQUAT(q));
    MSF_WARN_STREAM("q_wv " << STREAMQUAT(q_wv));

    Eigen::Matrix<double, 3, 1> p_vision = q_wv.conjugate().toRotationMatrix()
    * p_vc / scale - q.toRotationMatrix() * p_ic;

    //TODO (slynen): what if there is no initial position measurement? Then we
    // have to shift vision-world later on, before applying the first position
    // measurement.
    p = p_pos - q.toRotationMatrix() * p_ip;
    // p_wv = p - p_vision;  // Shift the vision frame so that it fits the position
    // measurement

    a_m = q.inverse() * g;			    /// Initial acceleration.

    //TODO (slynen) Fix this.
    //we want z from vision (we did scale init), so:
//    p(2) = p_vision(2);
//    p_wv(2) = 0;
//    position_handler_->adjustGPSZReference(p(2));

    // Prepare init "measurement"
    // True means that we will also set the initial sensor readings.
    shared_ptr < msf_core::MSF_InitMeasurement<EKFState_T>
    > meas(new msf_core::MSF_InitMeasurement<EKFState_T>(true));

    meas->SetStateInitValue < StateDefinition_T::p > (p_zero);
    meas->SetStateInitValue < StateDefinition_T::v > (v);
    meas->SetStateInitValue < StateDefinition_T::q > (q);
    meas->SetStateInitValue < StateDefinition_T::b_w > (b_w);
    meas->SetStateInitValue < StateDefinition_T::b_a > (b_a);
    meas->SetStateInitValue < StateDefinition_T::L
    > (Eigen::Matrix<double, 1, 1>::Constant(scale));
    meas->SetStateInitValue < StateDefinition_T::q_wv > (q_wv);
    meas->SetStateInitValue < StateDefinition_T::p_wv > (p_wv);
    meas->SetStateInitValue < StateDefinition_T::q_ic > (q_ic);
    meas->SetStateInitValue < StateDefinition_T::p_ic > (p_ic);
    meas->SetStateInitValue < StateDefinition_T::p_ip > (p_ip);
    meas->SetStateInitValue < StateDefinition_T::b_p > (b_p);


    meas->SetStateInitValue < StateDefinition_T::L_2
    > (Eigen::Matrix<double, 1, 1>::Constant(0.12434));
    meas->SetStateInitValue < StateDefinition_T::q_wv_2 > (q_wv_2);
    meas->SetStateInitValue < StateDefinition_T::p_wv_2 > (p_wv_2);
    meas->SetStateInitValue < StateDefinition_T::q_ic_2 > (q_ic_2);
    meas->SetStateInitValue < StateDefinition_T::p_ic_2 > (p_ic_2);


    SetStateCovariance(meas->GetStateCovariance());  // Call my set P function.
    meas->Getw_m() = w_m;
    meas->Geta_m() = a_m;
    meas->time = ros::Time::now().toSec();

    // Call initialization in core.
    msf_core_->Init(meas);
  }

  // Prior to this call, all states are initialized to zero/identity.
  virtual void ResetState(EKFState_T& state) const {
    // Set scale to 1.
    Eigen::Matrix<double, 1, 1> scale;
    scale << 1.0;
    state.Set < StateDefinition_T::L > (scale);
  }
  virtual void InitState(EKFState_T& state) const {
    UNUSED(state);
  }

  virtual void CalculateQAuxiliaryStates(EKFState_T& state, double dt) const {
    const msf_core::Vector3 nqwvv = msf_core::Vector3::Constant(
      config_.pose_noise_q_wv);
    const msf_core::Vector3 npwvv = msf_core::Vector3::Constant(
      config_.pose_noise_p_wv);
    const msf_core::Vector3 nqicv = msf_core::Vector3::Constant(
      config_.pose_noise_q_ic);
    const msf_core::Vector3 npicv = msf_core::Vector3::Constant(
      config_.pose_noise_p_ic);
    const msf_core::Vector1 n_L = msf_core::Vector1::Constant(
      config_.pose_noise_scale);
    const msf_core::Vector1 nb_p = msf_core::Vector1::Constant(
        config_.press_noise_bias_p);


    const msf_core::Vector3 nqwvv_2 = msf_core::Vector3::Constant(
      config_.pose_noise_q_wv_2);
    const msf_core::Vector3 npwvv_2 = msf_core::Vector3::Constant(
      config_.pose_noise_p_wv_2);
    const msf_core::Vector3 nqicv_2 = msf_core::Vector3::Constant(
      config_.pose_noise_q_ic_2);
    const msf_core::Vector3 npicv_2 = msf_core::Vector3::Constant(
      config_.pose_noise_p_ic_2);
    const msf_core::Vector1 n_L_2 = msf_core::Vector1::Constant(
      config_.pose_noise_scale_2);


    // Compute the blockwise Q values and store them with the states,
    // these then get copied by the core to the correct places in Qd.
    state.GetQBlock<StateDefinition_T::L>() = (dt * n_L.cwiseProduct(n_L))
    .asDiagonal();
    state.GetQBlock<StateDefinition_T::q_wv>() =
    (dt * nqwvv.cwiseProduct(nqwvv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_wv>() =
    (dt * npwvv.cwiseProduct(npwvv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::q_ic>() =
    (dt * nqicv.cwiseProduct(nqicv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_ic>() =
    (dt * npicv.cwiseProduct(npicv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::b_p>() = 
    (dt * nb_p.cwiseProduct(nb_p)).asDiagonal();


    state.GetQBlock<StateDefinition_T::L_2>() = (dt * n_L_2.cwiseProduct(n_L_2))
    .asDiagonal();
    state.GetQBlock<StateDefinition_T::q_wv_2>() =
    (dt * nqwvv_2.cwiseProduct(nqwvv_2)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_wv_2>() =
    (dt * npwvv_2.cwiseProduct(npwvv_2)).asDiagonal();
    state.GetQBlock<StateDefinition_T::q_ic_2>() =
    (dt * nqicv_2.cwiseProduct(nqicv_2)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_ic_2>() =
    (dt * npicv_2.cwiseProduct(npicv_2)).asDiagonal();
  }

  virtual void SetStateCovariance(
    Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,
    EKFState_T::nErrorStatesAtCompileTime>& P) const {
    UNUSED(P);
    // Nothing, we only use the simulated cov for the core plus diagonal for the
    // rest.
  }

  virtual void AugmentCorrectionVector(
    Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& correction) const {
    UNUSED(correction);
  }


//USER Check healthy
  virtual void SanityCheckCorrection(
    EKFState_T& delaystate,
    const EKFState_T& buffstate,
    Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& correction) const {

    UNUSED(buffstate);
    UNUSED(correction);

    

    const EKFState_T& state = delaystate;
    if (state.Get<StateDefinition_T::L>()(0) < 0) {
      MSF_WARN_STREAM_THROTTLE(
        1,
        "Negative scale detected: " << state.Get<StateDefinition_T::L>()(0) << ". Correcting to 0.1");
      Eigen::Matrix<double, 1, 1> L_;
      L_ << 0.1;
      delaystate.Set < StateDefinition_T::L > (L_);
    }


    double time_now = ros::Time::now().toSec();
    double lasttime_pose = pose_handler_->GetLasttime();
    double lasttime_position = position_handler_->GetLasttime();
    // MSF_WARN_STREAM_THROTTLE(0.1,"Time " << time_now << "\t" << lasttime_pose);
    if(lasttime_pose!=0)
      if(time_now-lasttime_pose > 1) {
        MSF_WARN_STREAM_THROTTLE(0.5,"SLAM timeout >1s" << time_now << "\t" << lasttime_pose);
        Eigen::Matrix<double, 1, 1> L_;
        L_ << 1;
        delaystate.Set < StateDefinition_T::L > (L_);

        Eigen::Matrix<double, 3, 1> p_wv_;
        p_wv_.setZero();
        delaystate.Set < StateDefinition_T::p_wv > (p_wv_);

        Eigen::Quaternion<double>  q_wv_;
        q_wv_.setIdentity();
        delaystate.Set < StateDefinition_T::q_wv > (q_wv_);

      }
    if(lasttime_position!=0)
      if(time_now-lasttime_position > 1)
       { 
        MSF_WARN_STREAM_THROTTLE(0.5,"GPS timeout >1s" << time_now << "\t" << lasttime_position);
       }


  }
};
}
#endif  // POSITION_POSE_SENSOR_MANAGER_H
