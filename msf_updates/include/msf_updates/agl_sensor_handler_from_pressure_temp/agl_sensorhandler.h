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
#ifndef AGL_SENSOR_H
#define AGL_SENSOR_H

#include <queue>

#include <msf_core/msf_sensormanagerROS.h>
#include <sensor_msgs/Range.h>

#include <msf_updates/agl_sensor_handler/agl_measurement.h>

namespace msf_agl_sensor {
class AglSensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:

  Eigen::Matrix<double, 1, 1> z_p_;  ///< Agl measurement.
  double n_zp_;  ///< Agl measurement noise.
  ros::Subscriber subAgl_;
  void MeasurementCallback(const sensor_msgs::RangeConstPtr & msg);
 public:
  AglSensorHandler(
      msf_core::MSF_SensorManager<msf_updates::EKFState>& meas,
      std::string topic_namespace, std::string parameternamespace);
  // Used for the init.
  Eigen::Matrix<double, 1, 1> GetAglMeasurement() {
    return z_p_;
  }

  // Setters for configure values.
  void SetNoises(double n_zp);
};
}  // namespace msf_agl_sensor
#include "implementation/agl_sensorhandler.hpp"
#endif  // POSE_SENSOR_H
