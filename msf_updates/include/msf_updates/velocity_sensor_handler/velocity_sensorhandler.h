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
#ifndef VELOCITY_SENSOR_H_
#define VELOCITY_SENSOR_H_

#include <msf_core/msf_sensormanagerROS.h>

#include <mavros_msgs/OpticalFlowRad.h>
#include <mavros_msgs/Altitude.h>

namespace msf_velocity_sensor {

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
class VelocitySensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:
 
  Eigen::Matrix<double, 3, 1> z_p_;  ///< Velocity measurement.
  double n_zv_;  ///< Velocity measurement noise.
  double delay_;       ///< Delay to be subtracted from the ros-timestamp of
                       //the measurement provided by this sensor.
  double flow_minQ_;  //flow_minQ coefficient
  Eigen::Quaternion<double> qif_;

  ros::Subscriber subFlow_, subAgl_;

  double agl_sensor;

  bool use_fixed_covariance_;  ///< Use fixed covariance set by dynamic reconfigure.
  bool provides_absolute_measurements_;  ///< Does this sensor measure relative or absolute values.

  void ProcessVelocityMeasurement(
      const mavros_msgs::OpticalFlowRadConstPtr& msg);

  void MeasurementCallback(
    const mavros_msgs::OpticalFlowRadConstPtr & msg);

  void MeasurementAGLCallback(
    const mavros_msgs::AltitudeConstPtr & msg);
 public:
  typedef MEASUREMENT_TYPE measurement_t;
  VelocitySensorHandler(MANAGER_TYPE& meas, std::string topic_namespace,
                        std::string parameternamespace);
  // Used for the init.
  Eigen::Matrix<double, 3, 1> GetVelocityMeasurement() {
    return z_p_;
  }
  // Setters for configure values.
  void SetNoises(double n_zv);
  void SetDelay(double delay);
  void SetMinQ(double flow_minQ);
  void SetQif(Eigen::Quaternion<double> qif);

  double timestamp_previous_pose_;  ///< Timestamp of previous pose message to subsample messages.
    // Used for check alive
  double GetLasttime() {
    return timestamp_previous_pose_;
  }
  
};
}  // namespace msf_velocity_sensor

#include "implementation/velocity_sensorhandler.hpp"

#endif  // VELOCITY_SENSOR_H_
