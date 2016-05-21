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
#ifndef MAG_SENSORHANDLER_HPP_
#define MAG_SENSORHANDLER_HPP_
#include <msf_core/msf_types.h>
#include <msf_core/eigen_utils.h>
// #include <msf_core/gps_conversion.h>

namespace msf_mag_sensor {
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
MagSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MagSensorHandler(
    MANAGER_TYPE& meas, std::string topic_namespace,
    std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      n_zm_(1e-6),
      delay_(0) {
  ros::NodeHandle pnh("~/mag_sensor");
  pnh.param("mag_use_fixed_covariance", use_fixed_covariance_, false);
  pnh.param("mag_absolute_measurements", provides_absolute_measurements_,
            false);

  MSF_INFO_STREAM_COND(use_fixed_covariance_, "Mag sensor is using fixed "
                       "covariance");
  MSF_INFO_STREAM_COND(!use_fixed_covariance_, "Mag sensor is using "
                       "covariance from sensor");

  MSF_INFO_STREAM_COND(provides_absolute_measurements_, "Mag sensor is "
                       "handling measurements as absolute values");
  MSF_INFO_STREAM_COND(!provides_absolute_measurements_, "Mag sensor is "
                       "handling measurements as relative values");

  ros::NodeHandle nh("msf_updates");

  subPointStamped_ =
      nh.subscribe<geometry_msgs::Vector3Stamped>
  ("magvec_input", 20, &MagSensorHandler::MeasurementCallback, this);

  
  subPoseStamped_ =
      nh.subscribe<sensor_msgs::MagneticField>
  ("mag_input", 20, &MagSensorHandler::MeasurementCallback, this);


  z_m_.setZero();

}
  static bool referenceinit = true;//false;  //TODO (slynen): Dynreconf reset reference.

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void MagSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetNoises(
    double n_zm) {
  n_zm_ = n_zm;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void MagSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetDelay(
    double delay) {
  delay_ = delay;
}
// template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
// void MagSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetLatlon( bool referenceinit_) {
//   referenceinit = false;
// }

// template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
// void MagSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::AdjustGPSZReference(
//     double current_z) {
//   gpsConversion_.AdjustReference(z_m_(2) - current_z);
// }

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void MagSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::ProcessMagMeasurement(
    const sensor_fusion_comm::PointWithCovarianceStampedConstPtr& msg) {
  received_first_measurement_ = true;

  // Get the fixed states.
  int fixedstates = 0;
  static_assert(msf_updates::EKFState::nStateVarsAtCompileTime < 32, "Your state "
      "has more than 32 variables. The code needs to be changed here to have a "
      "larger variable to mark the fixed_states");
  // Do not exceed the 32 bits of int.

  if (!use_fixed_covariance_ && msg->covariance[0] == 0)  // Take covariance from sensor.
      {
    MSF_WARN_STREAM_THROTTLE(
        2, "Provided message type without covariance but set "
        "fixed_covariance=false at the same time. Discarding message.");
    return;
  }

  // Get all the fixed states and set flag bits.
  MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);

  // if (mngr) {
  //   if (mngr->Getcfg().mag_fixed_p_ip) {
  //     fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::p_ip;
  //   }
  // }

  shared_ptr < MEASUREMENT_TYPE
      > meas(
          new MEASUREMENT_TYPE(n_zm_, use_fixed_covariance_,
                               provides_absolute_measurements_, this->sensorID,
                               fixedstates));

  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_m_ = meas->z_m_;  // Store this for the init procedure.

  this->manager_.msf_core_->AddMeasurement(meas);
}




template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void MagSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const geometry_msgs::Vector3StampedConstPtr & msg) {
  this->SequenceWatchDog(msg->header.seq, subPointStamped_.getTopic());

  MSF_INFO_STREAM_ONCE(
      "*** mag sensor got first measurement from topic vec3"
          << this->topic_namespace_ << "/" << subPointStamped_.getTopic()
          << " ***");

  sensor_fusion_comm::PointWithCovarianceStampedPtr pointwCov(
      new sensor_fusion_comm::PointWithCovarianceStamped);
  pointwCov->header = msg->header;
  pointwCov->point.x = msg->vector.x;
  pointwCov->point.y = msg->vector.y;
  pointwCov->point.z = msg->vector.z;
  // pointwCov->covariance[0] = msg->pose.covariance[0];
  // pointwCov->covariance[4] = msg->pose.covariance[7];
  // pointwCov->covariance[8] = msg->pose.covariance[14];
  ProcessMagMeasurement(pointwCov);
}


template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void MagSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const sensor_msgs::MagneticFieldConstPtr & msg) {
  this->SequenceWatchDog(msg->header.seq, subPoseStamped_.getTopic());

  MSF_INFO_STREAM_ONCE(
      "*** mag sensor got first measurement from topic magfield"
          << this->topic_namespace_ << "/" << subPoseStamped_.getTopic()
          << " ***");

  sensor_fusion_comm::PointWithCovarianceStampedPtr pointwCov(
      new sensor_fusion_comm::PointWithCovarianceStamped);
  pointwCov->header = msg->header;
  pointwCov->point.x = msg->magnetic_field.x;
  pointwCov->point.y = msg->magnetic_field.y;
  pointwCov->point.z = msg->magnetic_field.z;
  pointwCov->covariance[0] = msg->magnetic_field_covariance[0];
  pointwCov->covariance[4] = msg->magnetic_field_covariance[4];
  pointwCov->covariance[8] = msg->magnetic_field_covariance[8];
  ProcessMagMeasurement(pointwCov);
}


}  // namespace msf_mag_sensor
#endif  // MAG_SENSORHANDLER_HPP_
