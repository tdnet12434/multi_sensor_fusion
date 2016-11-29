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
#ifndef AGL_SENSOR_H_
#define AGL_SENSOR_H_

#include <msf_core/msf_sensormanagerROS.h>

#include <sensor_msgs/Range.h>
namespace msf_agl_sensor {

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
class AglSensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:
 
  Eigen::Matrix<double,1,1> z_sonar_;  ///< Agl measurement.
  double n_zsonar_;  ///< Agl measurement noise.


  ros::Subscriber subAgl_;


  bool use_fixed_covariance_;  ///< Use fixed covariance set by dynamic reconfigure.
  bool provides_absolute_measurements_;  ///< Does this sensor measure relative or absolute values.

  void ProcessAglMeasurement(
      const sensor_msgs::RangeConstPtr& msg);

  void MeasurementCallback(
    const sensor_msgs::RangeConstPtr & msg);

 public:
  typedef MEASUREMENT_TYPE measurement_t;
  AglSensorHandler(MANAGER_TYPE& meas, std::string topic_namespace,
                        std::string parameternamespace);
  // Used for the init.
  Eigen::Matrix<double,1,1> GetAglMeasurement() {
    return z_sonar_;
  }
  // Setters for configure values.
  void SetNoises(double n_zsonar);

  double timestamp_previous_pose_;  ///< Timestamp of previous pose message to subsample messages.
    // Used for check alive
  double GetLasttime() {
    return timestamp_previous_pose_;
  }
  
};
}  // namespace msf_agl_sensor

#include "implementation/agl_sensorhandler.hpp"

#endif  // AGL_SENSOR_H_
