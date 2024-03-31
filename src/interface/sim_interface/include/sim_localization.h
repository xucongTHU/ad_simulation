#ifndef SIM_INTERFACE_SIM_LOCALIZATION_INTERFACE_H
#define SIM_INTERFACE_SIM_LOCALIZATION_INTERFACE_H

#include <stdint.h>

#include <string>
#include <vector>

#include "sim_interface/sim_std.h"

namespace sim_localization {

struct LocalizationMeta {
  int64_t start_timestamp_us;
  int64_t end_timestamp_us;
  int64_t sensor_timestamp_us;
};
/**
 * @brief Localization Euler Angle
 * Roll/pitch/yaw that represents a rotation with intrinsic sequence z-x-y in
 * world coordinate(East / North / Up) The roll, in (-pi/2, pi/2), corresponds
 * to a rotation around the y-axis. The pitch, in [-pi, pi), corresponds to a
 * rotation around the x-axis. The yaw, in [-pi, pi), corresponds to a
 * rotation around the z-axis. The pitch is zero when the car is level and
 * positive when the nose is up. The roll is zero when the car is level and
 * positive when the left part is up.
 *  The yaw is zero when the car is facing North, and positive when facing
 *  West. The direction of rotation follows the right-hand rule.
 */
struct LocalizationEulerAngle {
  double roll;  ///<corresponds to a rotation around the y-axis
  double pitch; ///<corresponds to a rotation around the x-axis.
  double yaw;   ///<corresponds to a rotation around the z-axis.
};

struct LocalizationPose {
  uint64_t available;
  ///Pose of the world coordinate (ENU) in the map reference frame.
  sim_std::Point3d pose_info;
  ///Linear velocity of the VRF in the map reference frame.
  sim_std::Point3d linear_velocity_info;
  ///Linear acceleration of the ENU in the map reference frame.
  sim_std::Point3d linear_acceleration_info;
  ///Angular velocity of the vehicle in the map reference frame.
  sim_std::Point3d angular_velocity_info;
  ///Linear acceleration of the VRF in the vehicle reference frame.
  sim_std::Point3d linear_acceleration_vrf_info;
  ///Angular velocity of the VRF in the vehicle reference frame.
  sim_std::Point3d angular_velocity_vrf_info;
  ///Angular acceleration of the VRF in the vehicle reference frame.
  sim_std::Point3d angular_acceleration_vrf_info;

  sim_std::Quaternion orientation_info;
  ///The heading is zero when the car is facing East and positive when facing
  ///North.
  double heading;
  LocalizationEulerAngle angle;

};

struct LocalizationEstimation : public sim_std::MessageBase {
  sim_std::HeaderPOD header;
  uint64_t available;
  LocalizationMeta meta;
  LocalizationPose pose;
};

struct Gps : public sim_std::MessageBase {
  sim_std::HeaderPOD header;
  LocalizationMeta meta;
  LocalizationPose gps_info;
};

struct Imu : public sim_std::MessageBase {
  sim_std::HeaderPOD header;
  LocalizationMeta meta;
  LocalizationPose imu_info;
};

struct GpsUdp {
  double simTime;
  uint32_t simFrame;
  uint64_t available;
  LocalizationPose gps_info;
};

struct ImuUdp {
  double simTime;
  uint32_t simFrame;
  uint64_t available;
  LocalizationPose imu_info;
};

} // namespace sim_localization

#endif
