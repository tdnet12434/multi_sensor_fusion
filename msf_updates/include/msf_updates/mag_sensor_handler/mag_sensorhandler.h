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
#ifndef MAG_SENSOR_H_
#define MAG_SENSOR_H_

#include <msf_core/msf_sensormanagerROS.h>
// #include <geometry_msgs/PointStamped.h>
// #include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/MagneticField.h>
// #include <msf_core/gps_conversion.h>
#include <sensor_fusion_comm/PointWithCovarianceStamped.h>

namespace msf_mag_sensor {

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
class MagSensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:

  Eigen::Matrix<double, 3, 1> z_m_;  ///< Mag measurement.
  double n_zm_;  ///< Mag measurement noise.
  double delay_;       ///< Delay to be subtracted from the ros-timestamp of
                       //the measurement provided by this sensor.

  ros::Subscriber subPointStamped_;
  ros::Subscriber subPoseStamped_;
  // ros::Subscriber subTransformStamped_;
  // ros::Subscriber subNavSatFix_;
  // msf_core::GPSConversion gpsConversion_;

  bool use_fixed_covariance_;  ///< Use fixed covariance set by dynamic reconfigure.
  bool provides_absolute_measurements_;  ///< Does this sensor measure relative or absolute values.

  void ProcessMagMeasurement(
      const sensor_fusion_comm::PointWithCovarianceStampedConstPtr& msg);
  void MeasurementCallback(const geometry_msgs::Vector3StampedConstPtr & msg);
  void MeasurementCallback(const sensor_msgs::MagneticFieldConstPtr & msg);
  // void MeasurementCallback(const geometry_msgs::TransformStampedConstPtr & msg);
  // void MeasurementCallback(const sensor_msgs::NavSatFixConstPtr& msg);

 public:
  typedef MEASUREMENT_TYPE measurement_t;
  MagSensorHandler(MANAGER_TYPE& meas, std::string topic_namespace,
                        std::string parameternamespace);
  // Used for the init.
  Eigen::Matrix<double, 3, 1> GetMagMeasurement() {
    return z_m_;
  }
  // Setters for configure values.
  void SetNoises(double n_zm);
  void SetDelay(double delay);
  // void SetLatlon( bool referenceinit_);
  // void AdjustGPSZReference(double current_z);
};
}  // namespace msf_mag_sensor

#include "implementation/mag_sensorhandler.hpp"

#endif  // MAG_SENSOR_H_
