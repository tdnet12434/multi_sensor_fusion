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
#ifndef VELOCITY_SENSORHANDLER_HPP_
#define VELOCITY_SENSORHANDLER_HPP_
#include <msf_core/msf_types.h>
#include <msf_core/eigen_utils.h> 
 
namespace msf_velocity_sensor {
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
VelocitySensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::VelocitySensorHandler(
    MANAGER_TYPE& meas, std::string topic_namespace,
    std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      n_zv_(1e-6),
      delay_(0),
      drag_(0.1) { 
  ros::NodeHandle pnh("~/velocity_sensor");
  pnh.param("velocity_use_fixed_covariance", use_fixed_covariance_, false);
  pnh.param("velocity_absolute_measurements", provides_absolute_measurements_,
            false);

  MSF_INFO_STREAM_COND(use_fixed_covariance_, "Velocity sensor is using fixed "
                       "covariance");
  MSF_INFO_STREAM_COND(!use_fixed_covariance_, "Velocity sensor is using "
                       "covariance from sensor");

  MSF_INFO_STREAM_COND(provides_absolute_measurements_, "Velocity sensor is "
                       "handling measurements as absolute values");
  MSF_INFO_STREAM_COND(!provides_absolute_measurements_, "Velocity sensor is "
                       "handling measurements as relative values");

  ros::NodeHandle nh("msf_updates");

  subImu_ =
      nh.subscribe<sensor_msgs::Imu>
  ("imu_input", 20, &VelocitySensorHandler::MeasurementCallback, this);


  z_v_.setZero();

}
  static bool referenceinit = false;  //TODO (slynen): Dynreconf reset reference.

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocitySensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetNoises(
    double n_zv) {
  n_zv_ = n_zv;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocitySensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetDelay(
    double delay) {
  delay_ = delay;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocitySensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetDrag(
    double drag) {
  drag_ = drag;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocitySensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::ProcessVelocityMeasurement(
    const sensor_msgs::ImuConstPtr& msg) {
  received_first_measurement_ = true;

  // Get the fixed states.
  int fixedstates = 0;
  static_assert(msf_updates::EKFState::nStateVarsAtCompileTime < 32, "Your state "
      "has more than 32 variables. The code needs to be changed here to have a "
      "larger variable to mark the fixed_states");
  // Do not exceed the 32 bits of int.

  if (!use_fixed_covariance_ && msg->linear_acceleration_covariance[0] == 0)  // Take covariance from sensor.
      {
    MSF_WARN_STREAM_THROTTLE(
        2, "Provided message type without covariance but set "
        "fixed_covariance=false at the same time. Discarding message.");
    return;
  }

  // Get all the fixed states and set flag bits.
  MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);

  if (mngr) {
    if (mngr->Getcfg().position_fixed_p_ip) {
      fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::p_ip;
    }
  }

  shared_ptr < MEASUREMENT_TYPE
      > meas(
          new MEASUREMENT_TYPE(n_zv_, use_fixed_covariance_,
                               provides_absolute_measurements_, this->sensorID,
                               fixedstates,drag_));

  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_v_ = meas->z_v_;  // Store this for the init procedure.

  this->manager_.msf_core_->AddMeasurement(meas);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocitySensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const sensor_msgs::ImuConstPtr & msg) {
  this->SequenceWatchDog(msg->header.seq, subImu_.getTopic());

  MSF_INFO_STREAM_ONCE(
      "*** velocity sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subImu_.getTopic()
          << " ***");

  sensor_msgs::ImuPtr pointwCov(
      new sensor_msgs::Imu);
  pointwCov->header = msg->header;
  pointwCov->linear_acceleration = msg->linear_acceleration;
  // printf("%f %f %f\n",msg->linear_acceleration.x,z_v_(1,0),z_v_(2,0));
  double time_now = msg->header.stamp.toSec();
  timestamp_previous_pose_ = time_now;
  
  ProcessVelocityMeasurement(pointwCov);
}

}  // namespace msf_position_sensor
#endif  // VELOCITY_SENSORHANDLER_HPP_
