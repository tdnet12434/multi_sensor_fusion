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
#include <msf_core/eigen_utils.h>
#ifndef AGL_SENSORHANDLER_HPP_
#define AGL_SENSORHANDLER_HPP_

namespace msf_agl_sensor {
AglSensorHandler::AglSensorHandler(
    msf_core::MSF_SensorManager<msf_updates::EKFState>& meas,
    std::string topic_namespace, std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      n_zp_(0.01) {
  ros::NodeHandle pnh("~/agl_sensor");
  ros::NodeHandle nh("msf_updates");
  subAgl_ =
      nh.subscribe<sensor_msgs::Range>
      ("agl_height", 20, &AglSensorHandler::MeasurementCallback, this);


}

void AglSensorHandler::SetNoises(double n_zp) {
  n_zp_ = n_zp;
}






void AglSensorHandler::MeasurementCallback(
    const sensor_msgs::RangeConstPtr & msg) {

  received_first_measurement_ = true;


  this->SequenceWatchDog(msg->header.seq, subAgl_.getTopic());
  MSF_INFO_STREAM_ONCE(
      "*** agl sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subAgl_.getTopic()
          << " ***");
  


  shared_ptr<agl_measurement::AglMeasurement> meas(
      new agl_measurement::AglMeasurement(n_zp_, true,
                                                    this->sensorID));
  meas->MakeFromSensorReading(msg_av, msg_av->header.stamp.toSec());

  z_p_ = meas->z_p_;  // Store this for the init procedure.


  this->manager_.msf_core_->AddMeasurement(meas);
}
}  // namespace msf_agl_sensor
#endif  // AGL_SENSORHANDLER_HPP_
