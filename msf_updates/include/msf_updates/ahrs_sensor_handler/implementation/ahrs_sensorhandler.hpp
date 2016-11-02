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
#ifndef AHRS_SENSORHANDLER_HPP_
#define AHRS_SENSORHANDLER_HPP_
#include <msf_core/msf_types.h>
#include <msf_core/eigen_utils.h>

namespace msf_ahrs_sensor {
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
AhrsSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::AhrsSensorHandler(
    MANAGER_TYPE& meas, std::string topic_namespace,
    std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      n_zq_(0.01),
      timestamp_previous_pose_(0) {
  ros::NodeHandle pnh("~/ahrs_sensor");
  pnh.param("ahrs_use_fixed_covariance", use_fixed_covariance_, true);
  pnh.param("ahrs_absolute_measurements", provides_absolute_measurements_,
            true);
 
  MSF_INFO_STREAM_COND(use_fixed_covariance_, "Ahrs sensor is using fixed "
                       "covariance");
  MSF_INFO_STREAM_COND(!use_fixed_covariance_, "Ahrs sensor is using "
                       "covariance from sensor");

  MSF_INFO_STREAM_COND(provides_absolute_measurements_, "Ahrs sensor is "
                       "handling measurements as absolute values");
  MSF_INFO_STREAM_COND(!provides_absolute_measurements_, "Ahrs sensor is "
                       "handling measurements as relative values");

  ros::NodeHandle nh("msf_updates");

  subImu_ =
      nh.subscribe<sensor_msgs::Imu>
  ("imu_input", 5, &AhrsSensorHandler::MeasurementCallback, this);



  z_q_.setIdentity();

}
  // static bool referenceinit = false;  //TODO (slynen): Dynreconf reset reference.

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void AhrsSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetNoises(
    double n_zq) {
  n_zq_ = n_zq;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void AhrsSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::ProcessAhrsMeasurement(
    const sensor_msgs::ImuConstPtr& msg) {
  received_first_measurement_ = true;

  // Get the fixed states.
  int fixedstates = 0;
  static_assert(msf_updates::EKFState::nStateVarsAtCompileTime < 32, "Your state "
      "has more than 32 variables. The code needs to be changed here to have a "
      "larger variable to mark the fixed_states");
  // Do not exceed the 32 bits of int.

  if (!use_fixed_covariance_  && 0/*&& msg->linear_acceleration_covariance[0] == 0*/)  // Take covariance from sensor.
      {
    MSF_WARN_STREAM_THROTTLE(
        2, "Provided message type without covariance but set "
        "fixed_covariance=false at the same time. Discarding message.");
    return;
  }


  shared_ptr < MEASUREMENT_TYPE
      > meas(
          new MEASUREMENT_TYPE(n_zq_, use_fixed_covariance_,
                               provides_absolute_measurements_, 
                               this->sensorID,
                               fixedstates));

  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec());

  z_q_ = meas->z_q_;  // Store this for the init procedure.

  this->manager_.msf_core_->AddMeasurement(meas);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void AhrsSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const sensor_msgs::ImuConstPtr & msg) {
  this->SequenceWatchDog(msg->header.seq, subImu_.getTopic());

  if (msg->header.seq % 10 != 0) {
    return;
  }
  
  MSF_INFO_STREAM_ONCE(
      "*** ahrs sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subImu_.getTopic()
          << " ***");

   double time_now = msg->header.stamp.toSec();
  timestamp_previous_pose_ = time_now;

  ProcessAhrsMeasurement(msg);
}

}  // namespace msf_position_sensor
#endif  // AHRS_SENSORHANDLER_HPP_
