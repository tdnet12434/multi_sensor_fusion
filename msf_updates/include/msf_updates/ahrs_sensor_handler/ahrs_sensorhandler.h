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
#ifndef AHRS_SENSOR_H_
#define AHRS_SENSOR_H_

#include <msf_core/msf_sensormanagerROS.h>

#include <sensor_msgs/Imu.h>
namespace msf_ahrs_sensor {

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
class AhrsSensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:
 
  Eigen::Quaternion<double> z_q_;  ///< Ahrs measurement.
  double n_zq_;  ///< Ahrs measurement noise.


  ros::Subscriber subImu_;


  bool use_fixed_covariance_;  ///< Use fixed covariance set by dynamic reconfigure.
  bool provides_absolute_measurements_;  ///< Does this sensor measure relative or absolute values.

  void ProcessAhrsMeasurement(
      const sensor_msgs::ImuConstPtr& msg);

  void MeasurementCallback(
    const sensor_msgs::ImuConstPtr & msg);

 public:
  typedef MEASUREMENT_TYPE measurement_t;
  AhrsSensorHandler(MANAGER_TYPE& meas, std::string topic_namespace,
                        std::string parameternamespace);
  // Used for the init.
  Eigen::Quaternion<double> GetAhrsMeasurement() {
    return z_q_;
  }
  // Setters for configure values.
  void SetNoises(double n_zq);

  double timestamp_previous_pose_;  ///< Timestamp of previous pose message to subsample messages.
    // Used for check alive
  double GetLasttime() {
    return timestamp_previous_pose_;
  }
  
};
}  // namespace msf_ahrs_sensor

#include "implementation/ahrs_sensorhandler.hpp"

#endif  // AHRS_SENSOR_H_
