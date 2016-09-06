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
      n_zv_(0.01),
      delay_(0),
      flow_minQ_(75),
      qif_(Eigen::Quaternion<double>(1,0,0,0)),
      timestamp_previous_pose_(0) {
  ros::NodeHandle pnh("~/velocity_sensor");
  pnh.param("velocity_use_fixed_covariance", use_fixed_covariance_, true);
  pnh.param("velocity_absolute_measurements", provides_absolute_measurements_,
            true);
 
  MSF_INFO_STREAM_COND(use_fixed_covariance_, "Velocity sensor is using fixed "
                       "covariance");
  MSF_INFO_STREAM_COND(!use_fixed_covariance_, "Velocity sensor is using "
                       "covariance from sensor");

  MSF_INFO_STREAM_COND(provides_absolute_measurements_, "Velocity sensor is "
                       "handling measurements as absolute values");
  MSF_INFO_STREAM_COND(!provides_absolute_measurements_, "Velocity sensor is "
                       "handling measurements as relative values");

  ros::NodeHandle nh("msf_updates");

  subFlow_ =
      nh.subscribe<mavros_msgs::OpticalFlowRad>
  ("flow_input", 20, &VelocitySensorHandler::MeasurementCallback, this);

  subAgl_ =
      nh.subscribe<mavros_msgs::Altitude>
  ("agl_input", 20, &VelocitySensorHandler::MeasurementAGLCallback, this);


  z_p_.setZero();

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
void VelocitySensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetMinQ(
    double flow_minQ) {
  flow_minQ_ = flow_minQ;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocitySensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetQif(
    Eigen::Quaternion<double> qif) {
  qif_ = qif;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocitySensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::ProcessVelocityMeasurement(
    const mavros_msgs::OpticalFlowRadConstPtr& msg) {
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

  // Get all the fixed states and set flag bits.
  MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);

  if (mngr) {
    if (mngr->Getcfg().flow_is_absolute) {
      fixedstates = 1;
    }
  }

  shared_ptr < MEASUREMENT_TYPE
      > meas(
          new MEASUREMENT_TYPE(n_zv_, use_fixed_covariance_,
                               provides_absolute_measurements_, 
                               this->sensorID,
                               fixedstates,
                               flow_minQ_,
                               qif_));

  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_p_ = meas->z_p_;  // Store this for the init procedure.

  this->manager_.msf_core_->AddMeasurement(meas);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocitySensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const mavros_msgs::OpticalFlowRadConstPtr & msg) {
  this->SequenceWatchDog(msg->header.seq, subFlow_.getTopic());

  MSF_INFO_STREAM_ONCE(
      "*** velocity sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subFlow_.getTopic()
          << " ***");




  mavros_msgs::OpticalFlowRadPtr flow_msg(
      new mavros_msgs::OpticalFlowRad());
  // Fixed covariance will be set in measurement class -> MakeFromSensorReadingImpl.
  flow_msg->header = msg->header;
  flow_msg->integration_time_us = msg->integration_time_us;
  flow_msg->integrated_x = msg->integrated_x;
  flow_msg->integrated_y = msg->integrated_y;
  flow_msg->integrated_xgyro = msg->integrated_xgyro;
  flow_msg->integrated_ygyro = msg->integrated_ygyro;
  flow_msg->integrated_zgyro = msg->integrated_zgyro;
  flow_msg->temperature = msg->temperature;
  flow_msg->quality = msg->quality;
  flow_msg->time_delta_distance_us = msg->time_delta_distance_us;

  flow_msg->distance = agl_sensor;

  if(flow_msg->quality < flow_minQ_) return;
  if(agl_sensor < 0.3) return;
  if(agl_sensor > 4) return;

   double time_now = msg->header.stamp.toSec();
  timestamp_previous_pose_ = time_now;

  ProcessVelocityMeasurement(flow_msg);
}
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocitySensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementAGLCallback(
    const mavros_msgs::AltitudeConstPtr & msg) {
  this->SequenceWatchDog(msg->header.seq, subAgl_.getTopic());

  MSF_INFO_STREAM_ONCE(
      "*** velocity sensor got first agl measurement from topic "
          << this->topic_namespace_ << "/" << subAgl_.getTopic()
          << " ***");

  agl_sensor = msg->relative;
}
}  // namespace msf_position_sensor
#endif  // VELOCITY_SENSORHANDLER_HPP_
