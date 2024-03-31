#ifndef ad_INTERFACE_ad_LOCALIZATION_INTERFACE_H
#define ad_INTERFACE_ad_LOCALIZATION_INTERFACE_H

#include <stdint.h>

#include "ad_interface/ad_sensor.h"
#include "ad_interface/ad_std.h"

namespace ad_localization {

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
  ///Pose of the vehicle reference point (VRP) in the map reference frame.
  ad_std::PoseWithCovariance pose_info;
  ///Linear velocity of the VRP in the map reference frame.
  ad_std::LinearVelocityWithCovariance linear_velocity_info;
  ///Linear acceleration of the VRP in the map reference frame.
  ad_std::LinearAccelerationWithCovariance linear_acceleration_info;
  ///Angular velocity of the vehicle in the map reference frame.
  ad_std::AngularVelocityWithCovariance angular_velocity_info;
  ///Linear acceleration of the VRP in the vehicle reference frame.
  ad_std::LinearAccelerationWithCovariance linear_acceleration_vrf_info;
  ///Angular velocity of the VRP in the vehicle reference frame.
  ad_std::AngularVelocityWithCovariance angular_velocity_vrf_info;
  ///The heading is zero when the car is facing East and positive when facing
  ///North.
  double heading;
  LocalizationEulerAngle angle;

  enum : uint64_t {
    POSE = 1 << 0,
    LINEAR_VELOCITY = 1 << 1,
    LINEAR_ACCELERATION = 1 << 2,
    ANGLE_VELOCITY = 1 << 3,
    LINEAR_ACCELERATION_VRF = 1 << 4,
    ANGLE_VELOCITY_VRF = 1 << 5,
    HEADING = 1 << 6,
    ANGLE = 1 << 7,
  };
};

/**
 * @brief LiDAR-based loclaization module status
 */
enum class LocalLidarStatus : uint8_t {
  MSF_LOCAL_LIDAR_NORMAL = 0,  ///<Localization result satisfy threshold
  MSF_LOCAL_LIDAR_MAP_MISSING, ///<Can't find localization map (config.xml)
  MSF_LOCAL_LIDAR_EXTRINSICS_MISSING, ///<Missing extrinsic parameters
  MSF_LOCAL_LIDAR_MAP_LOADING_FAILED, ///<Fail to load localization map
  MSF_LOCAL_LIDAR_NO_OUTPUT,  ///<No output (comparing to timestamp of imu msg)
  MSF_LOCAL_LIDAR_OUT_OF_MAP, ///<Coverage of online pointcloud and map is lower
                              ///<than threshold
  MSF_LOCAL_LIDAR_NOT_GOOD,   ///<Localization result do not meet threshold
  MSF_LOCAL_LIDAR_UNDEFINED_STATUS, ///<others
};

enum class LocalLidarQuality : uint8_t {
  MSF_LOCAL_LIDAR_VERY_GOOD = 0,
  MSF_LOCAL_LIDAR_GOOD,
  MSF_LOCAL_LIDAR_NOT_BAD,
  MSF_LOCAL_LIDAR_BAD,
};

enum class LocalVisualQuality : uint8_t {
  MSF_LOCAL_VISUAL_VERY_GOOD = 0,
  MSF_LOCAL_VISUAL_GOOD,
  MSF_LOCAL_VISUAL_NOT_BAD,
  MSF_LOCAL_VISUAL_BAD,
};

enum class LocalLaneLineQuality : uint8_t {
  MSF_LOCAL_LANE_LINE_VERY_GOOD = 0,
  MSF_LOCAL_LANE_LINE_GOOD,
  MSF_LOCAL_LANE_LINE_NOT_BAD,
  MSF_LOCAL_LANE_LINE_BAD,
};

/**
 * @brief LiDAR-based loclaization module status
 * LiDAR-based localization result check (the difference between lidar and sins
 * result
 */
enum class LocalLidarConsistency : uint8_t {
  MSF_LOCAL_LIDAR_CONSISTENCY_00 =
      0,                          ///<The difference is less than threshold 1
  MSF_LOCAL_LIDAR_CONSISTENCY_01, ///<The difference is bigger than threshold 1
                                  ///<but less than threshold 2
  MSF_LOCAL_LIDAR_CONSISTENCY_02, ///<The difference is bigger than threshold 2
  MSF_LOCAL_LIDAR_CONSISTENCY_03, ///<others
};

///<GNSS-based localization result check (the difference between GNSS and sins
///<result)
enum class GnssConsistency : uint8_t {
  MSF_GNSS_CONSISTENCY_00 = 0, ///<The difference is less than threshold 1
  MSF_GNSS_CONSISTENCY_01,     ///<The difference is bigger than threshold 1 but
                               ///<less than threshold 2
  MSF_GNSS_CONSISTENCY_02,     ///<The difference is bigger than threshold 2
  MSF_GNSS_CONSISTENCY_03,     ///<others
};

enum class LocalVisualConsistency : uint8_t {
  MSF_LOCAL_VISUAL_CONSISTENCY_00 =
      0,                           ///< The difference is less than threshold 1
  MSF_LOCAL_VISUAL_CONSISTENCY_01, ///< The difference is bigger than threshold
                                   ///< 1 but less than threshold 2
  MSF_LOCAL_VISUAL_CONSISTENCY_02, ///< The difference is bigger than threshold
                                   ///< 2
  MSF_LOCAL_VISUAL_CONSISTENCY_03, ///< others
};

enum class LocalLaneLineConsistency : uint8_t {
  MSF_LOCAL_LANE_LINE_CONSISTENCY_00 =
      0, ///< The difference is less than threshold 1
  MSF_LOCAL_LANE_LINE_CONSISTENCY_01, ///< The difference is bigger than
                                      ///< threshold 1 but less than threshold 2
  MSF_LOCAL_LANE_LINE_CONSISTENCY_02, ///< The difference is bigger than
                                      ///< threshold
                                      ///< 2
  MSF_LOCAL_LANE_LINE_CONSISTENCY_03, ///< others
};

enum class GnssPositionType : uint8_t {
  ///No solution
  NONE = 0,
  ///Position has been fixed by the FIX POSITION command or by position
  ///averaging
  FIXEDPOS = 1,
  ///Position has been fixed by the FIX HEIGHT, or FIX AUTO,command or by
  ///position averaging
  FIXEDHEIGHT = 2,
  ///Solution from floating point carrier phase anbiguities
  FLOATCONV = 4,
  ///Solution from wide-lane ambiguities
  WIDELANE = 5,
  ///Solution from narrow-lane ambiguities
  NARROWLANE = 6,
  ///Velocity computed using instantaneous Doppler
  DOPPLER_VELOCITY = 8,
  ///Single point position
  SINGLE = 16,
  ///Pseudorange differential solution
  PSRDIFF = 17,
  ///Solution calculated using corrections from an SBAS
  WAAS = 18,
  ///Propagated by a Kalman filter without new observations
  PROPOGATED = 19,
  ///OmniSTAR VBS position
  OMNISTAR = 20,
  ///Floating L1 albiguity solution
  L1_FLOAT = 32,
  ///Floating ionospheric free ambiguity solution
  IONOFREE_FLOAT = 33,
  ///Floating narrow-lane anbiguity solution
  NARROW_FLOAT = 34,
  ///Integer L1 ambiguity solution
  L1_INT = 48,
  ///Integer wide-lane ambiguity solution
  WIDE_INT = 49,
  ///Integer narrow-lane ambiguity solution
  NARROW_INT = 50,
  ///RTK status where RTK filter is directly initialized from the INS filter
  RTK_DIRECT_INS = 51,
  ///INS calculated position corrected for the antenna
  INS_SBAS = 52,
  ///INS pseudorange single point solution - no DGPS corrections
  INS_PSRSP = 53,
  ///INS pseudorange differential solution
  INS_PSRDIFF = 54,
  ///INS RTK float point ambiguities solution
  INS_RTKFLOAT = 55,
  ///INS RTK fixed ambiguities solution
  INS_RTKFIXED = 56,
  ///INS OmniSTAR VBS solution
  INS_OMNISTAR = 57,
  ///INS OmniSTAR high precision solution
  INS_OMNISTAR_HP = 58,
  ///INS OmniSTAR extra precision solution
  INS_OMNISTAR_XP = 59,
  ///OmniSTAR high precision
  OMNISTAR_HP = 64,
  ///OmniSTAR extra precision
  OMNISTAR_XP = 65,
  ///Precise Point Position(PPP) solution converging
  PPP_CONVERGING = 68,
  ///Precise Point Position(PPP)solution
  PPP = 69,
  ///INS NovAtel CORRECT Precise Point Position(PPP) solution converging
  INS_PPP_Converging = 73,
  ///INS NovAtel CORRECT Precise Point Position(PPP) solution
  INS_PPP = 74,
  ///Gnss position message loss
  MSG_LOSS = 91,
};

/**
 * @brief The running status of localization module
 */
enum class MsfRunningStatus : uint8_t {
  MSF_SOL_LIDAR_GNSS = 0,
  MSF_SOL_X_GNSS,
  MSF_SOL_LIDAR_X,
  MSF_SOL_LIDAR_XX,
  MSF_SOL_LIDAR_XXX,
  MSF_SOL_X_X,
  MSF_SOL_X_XX,
  MSF_SOL_X_XXX,
  MSF_SSOL_LIDAR_GNSS,
  MSF_SSOL_X_GNSS,
  MSF_SSOL_LIDAR_X,
  MSF_SSOL_LIDAR_XX,
  MSF_SSOL_LIDAR_XXX,
  MSF_SSOL_X_X,
  MSF_SSOL_X_XX,
  MSF_SSOL_X_XXX,
  MSF_NOSOL_LIDAR_GNSS,
  MSF_NOSOL_X_GNSS,
  MSF_NOSOL_LIDAR_X,
  MSF_NOSOL_LIDAR_XX,
  MSF_NOSOL_LIDAR_XXX,
  MSF_NOSOL_X_X,
  MSF_NOSOL_X_XX,
  MSF_NOSOL_X_XXX,
  MSF_RUNNING_INIT,
};

struct MsfStatus {
  uint64_t available;
  LocalLidarConsistency local_lidar_consistency;
  GnssConsistency gnss_consistency;
  LocalVisualConsistency local_visual_consistency;
  LocalLaneLineConsistency local_lane_line_consistency;
  LocalLidarStatus local_lidar_status;
  LocalLidarQuality local_lidar_quality;
  LocalVisualQuality local_visual_quality;
  LocalLaneLineQuality local_lane_line_quality;
  GnssPositionType gnsspos_position_type;
  MsfRunningStatus msf_running_status;

  enum : uint64_t {
    LOCAL_LIDAR_CONSISTENCY = 1 << 0,
    GNSS_CONSISTENCY = 1 << 1,
    LOCAL_VISUAL_CONSISTENCY = 1 << 2,
    LOCAL_LANE_LINE_CONSISTENCY = 1 << 3,
    LOCAL_LIDAR_STATUS = 1 << 4,
    LOCAL_LIDAR_QUALITY = 1 << 5,
    LOCAL_VISUAL_QUALITY = 1 << 6,
    LOCAL_LANE_LINE_QUALITY = 1 << 7,
    GNSS_POSTION_TYPE = 1 << 8,
    MSF_RUNNING_STATUS = 1 << 9,
  };
};

enum class AbsoluteLocalizationStatus : uint8_t {
  LOCAL_GOOD = 0,
  LOCAL_BAD,
};

enum class RoadMatchStatus : uint8_t {
  STATUS_OFFROAD = 0,
  STATUS_ONROAD,
};

struct LocalizationEstimation : public ad_std::MessageBase {
  ad_std::HeaderPOD header;
  LocalizationMeta meta;
  uint64_t available;
  LocalizationPose pose;
  // MsfStatus msf_status; //maxinjun 20231122 comment unused code 
  ///0: localization is reliable 1: localization may diverge,and module needs
  ///to be called for further judgment
  AbsoluteLocalizationStatus status;
  LocalizationPose relative_pose; // from cur body coordinate to the starting
  ad_std::PoseWithCovariance
      transform_pose; // from map coordinate to to the starting
  uint8_t localization_level;
  uint64_t lane_id; //global lane id in map
  uint64_t link_id; //global road id in map
  uint64_t path_id; //global path id in map
  RoadMatchStatus road_status;
  float lane_match_confidence; //the confidence of car matched lane, the value is from 0 to 1
  float road_match_confidence; //the confidence of car matched road, the value is from 0 to 1
  float dis_to_link_start; //the distance to current link start point(unit: m)
  uint8_t lane_number; //from right(number:0) to left
  float dis_to_global_start;//the distance to route start point(unit:m)
  ad_std::Point3d lla;//WGS84 x:longitude(unit:deg),y:latitude(unit:deg),z:altitude(unit:m)
  char reserved_info[STD_RESERVED_INFO_SIZE];

  enum : uint64_t {
    LOCALIZATION_ESTIMATION_POSE = 1 << 0,
    LOCALIZATION_ESTIMATION_MSF_STATUS = 1 << 1,
    LOCALIZATION_ESTIMATION_STATUS = 1 << 2,
    LOCALIZATION_ESTIMATION_RELATIVE_POSE = 1 << 3,
    LOCALIZATION_ESTIMATION_TRANSFORM_POSE = 1 << 4,
    LOCALIZATION_ESTIMATION_LOCALIZATION_LEVEL = 1 << 5,
    LOCALIZATION_ESTIMATION_LANE_ID = 1 << 6,
    LOCALIZATION_ESTIMATION_LINK_ID = 1 << 7,
    LOCALIZATION_ESTIMATION_PATH_ID = 1 << 8,
    LOCALIZATION_ESTIMATION_ROAD_STATUS = 1 << 9,
    LOCALIZATION_ESTIMATION_LANE_MATCH_CONFIDENCE = 1 << 10,
    LOCALIZATION_ESTIMATION_ROAD_MATCH_CONFIDENCE = 1 << 11,
    LOCALIZATION_ESTIMATION_DIS_TO_LINK_START = 1 << 12,
    LOCALIZATION_ESTIMATION_LANE_NUMBER = 1 << 13,
    LOCALIZATION_ESTIMATION_DIS_TO_GLOBAL_START = 1 << 14,
    LOCALIZATION_ESTIMATION_LLA = 1 << 15,
    LOCALIZATION_ESTIMATION_RESERVED_INFO = 1 << 16,
  };
};

} // namespace ad_localization

#endif
