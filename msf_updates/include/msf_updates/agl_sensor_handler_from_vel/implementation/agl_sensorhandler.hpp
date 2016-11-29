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
#ifndef AGL_SENSORHANDLER_HPP_
#define AGL_SENSORHANDLER_HPP_
#include <msf_core/msf_types.h>
#include <msf_core/eigen_utils.h>

namespace msf_agl_sensor {
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
AglSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::AglSensorHandler(
    MANAGER_TYPE& meas, std::string topic_namespace,
    std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, 
                                           topic_namespace,
                                           parameternamespace),

                                           n_zv_(0.01),
                                           timestamp_previous_pose_(0) {
  ros::NodeHandle pnh("~/agl_sensor");
  pnh.param("agl_use_fixed_covariance", use_fixed_covariance_, true);
  pnh.param("agl_absolute_measurements", provides_absolute_measurements_,
            true);
 
  MSF_INFO_STREAM_COND(use_fixed_covariance_, "Agl sensor is using fixed "
                       "covariance");
  MSF_INFO_STREAM_COND(!use_fixed_covariance_, "Agl sensor is using "
                       "covariance from sensor");

  MSF_INFO_STREAM_COND(provides_absolute_measurements_, "Agl sensor is "
                       "handling measurements as absolute values");
  MSF_INFO_STREAM_COND(!provides_absolute_measurements_, "Agl sensor is "
                       "handling measurements as relative values");

  ros::NodeHandle nh("msf_updates");

  subAgl_ =
      nh.subscribe<sensor_msgs::Range>
  ("agl_input", 10, &AglSensorHandler::MeasurementCallback, this);

  // subAgl_ =
  //     nh.subscribe<sensor_msgs::Range ("agl_input", 20, &AglSensorHandler::MeasurementAGLCallback, this);


  z_p_.setZero();

}
  // static bool referenceinit = false;  //TODO (slynen): Dynreconf reset reference.

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void AglSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetNoises(
    double n_zv) {
  n_zv_ = n_zv;
}




template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void AglSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::ProcessAglMeasurement(
    const sensor_msgs::RangeConstPtr& msg) {
  received_first_measurement_ = true;

  // Get the fixed states.
  int fixedstates = 0;


  shared_ptr < MEASUREMENT_TYPE
      > meas(
          new MEASUREMENT_TYPE(n_zv_, 
                               use_fixed_covariance_,
                               provides_absolute_measurements_, 
                               this->sensorID,
                               fixedstates));

  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec());

  z_p_ = meas->z_p_;  // Store this for the init procedure.

  this->manager_.msf_core_->AddMeasurement(meas);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void AglSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const sensor_msgs::RangeConstPtr & msg) {
  this->SequenceWatchDog(msg->header.seq, subAgl_.getTopic());

  MSF_INFO_STREAM_ONCE(
      "*** agl sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subAgl_.getTopic()
          << " ***");



  if(msg->range < msg->min_range || 
     msg->range > msg->max_range - 0.5)
      return;

  sensor_msgs::RangePtr range_msg(
      new sensor_msgs::Range());




  range_msg->header = msg->header;
  range_msg->range = msg->range;




   double time_now = msg->header.stamp.toSec();
  timestamp_previous_pose_ = time_now;

  ProcessAglMeasurement(range_msg);
}
// template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
// void AglSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementAGLCallback(
//     const sensor_msgs::Rangetr & msg) {
//   this->SequenceWatchDog(msg->header.seq, subAgl_.getTopic());

//   MSF_INFO_STREAM_ONCE(
//       "*** agl sensor got first agl measurement from topic "
//           << this->topic_namespace_ << "/" << subAgl_.getTopic()
//           << " ***");
//   // static uint16_t num_sen = 0;
//   // static float sum_sen = 0;
//   // static float offset = 0;
//   // if(num_sen<10) {
//   //   num_sen++;
//   //   sum_sen+=msg->bottom_clearance;
//   // }
//   // if(num_sen==10) {
//   //   offset = sum_sen/num_sen;
//   // }
//   agl_sensor = msg->bottom_clearance /*- offset*/;
// }
}  // namespace msf_position_sensor
#endif  // AGL_SENSORHANDLER_HPP_
