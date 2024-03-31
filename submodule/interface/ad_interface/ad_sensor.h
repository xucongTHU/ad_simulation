#ifndef ad_INTERFACE_ad_SENSOR_INTERFACE_H
#define ad_INTERFACE_ad_SENSOR_INTERFACE_H

#include <stdint.h>

#include <array>
#include <string>
#include <vector>

#include "ad_interface/ad_std.h"

namespace ad_sensor {

struct ImageMeta {
  int64_t sensor_timestamp_us;
  int64_t start_timestamp_us;
  int64_t finish_timestamp_us;
};

using ImageEncoding = ad_std::ImageEncoding;

struct Image : public ad_std::MessageBase {
  ad_std::Header header;
  ImageMeta meta;
  uint64_t available;
  std::string sensor_name; ///< camera sensor name
  uint32_t width;          ///< image columns
  uint32_t height;         ///< image rows
  ImageEncoding encoding;  ///< image color map:yuv,rgb,bgr
  uint8_t is_bigendian;    ///< device storage mode:bigendian or smallendian
  uint32_t step; ///< image storage step size:(image width)*(image channels)
  uint32_t size; ///< actual used size
  std::array<uint8_t, 1>
      data; ///< image data array,max image data: 3840x2160x3 bytes
  uint8_t *p_data;

  enum : uint64_t {
    IMAGE_SENSOR_NAME = 1 << 0,
    IMAGE_WIDTH = 1 << 1,
    IMAGE_HEIGHT = 1 << 2,
    IMAGE_ENCODING = 1 << 3,
    IMAGE_IS_BIGENDIAN = 1 << 4,
    IMAGE_STEP = 1 << 5,
    IMAGE_SIZE = 1 << 6,
    IMAGE_DATA = 1 << 7,
  };
};

struct CompressedImage {
  ad_std::Header header;
  ImageMeta meta;
  uint64_t available;
  std::string format;        ///< compress format:jpeg,h.264
  uint8_t quality;           ///< compress quality, 0~100
  std::vector<uint8_t> data; ///< compress data stream,data length is variable

  enum : uint64_t {
    COMPRESSED_IMAGE_FORMAT = 1 << 0,
    COMPRESSED_QUALITY = 1 << 1,
    COMPRESSED_IMAGE_DATA = 1 << 2,
  };
};

//************************************

/**
 * @brief point cloud meta
 *
 */
struct PointCloudMeta {
  int64_t sensor_timestamp_us;
  int64_t start_timestamp_us;
  int64_t finish_timestamp_us;
};

enum class LidarType : uint8_t {
  UNKNOW = 0,
  PANDAR128,
  PANDAR64,
  PANDAR16,
  AT128,
  M1,
};

enum class PcdPointLabel : uint8_t {
  UNKOWN = 0,
  NOT_ROI,
  ROI,
  GROUND,                 /**< ground surface */
  SKY,                    /**< point above a height threshold */
  NOISE,                  /**< noise below ground */
  CLUSTER_IGNORE,         /**< point in ignored clustered object */
  XYZ_NAN,                /**< xyz Not a Number */
  SELF,                   /**< point on ego car */
  TS_OUT_OF_BOUND,        /**< timestamp not in 100ms */
  DS_OUT_OF_BOUND_RAW,    /**<raw lidar data: distance out of range */
  DS_OUT_OF_BOUND_SENSOR, /**< compensated in sensor data: distance out of range
                           */
  ROI_INNER_EDGE,         /**< edge in ROI */
  ROI_OUTER_EDGE,         /**< edge out of ROI */
  DOWNSAMPLED,            /**< downsample*/
  WAIT_LIST,              /**<wait list */
  MAX                     /**< max */
};

/**
 * @brief pointXYZIRTL
 *
 */
struct PointXYZIRTL {
  float x;                       /**<x position */
  float y;                       /**<y position */
  float z;                       /**<z position */
  uint8_t intensity;             /**<intensity */
  uint16_t ring;                 /**<ring, laser */
  uint16_t timestamp_2us;        /**<timestamp unit 2*<us> */
  PcdPointLabel pcd_point_label; /**<point label, sky, ground, not roi*/
};

/**
 * @brief point cloud type array
 *
 */
struct PointCloudTypeArray {
  int64_t timestamp; /**<timestamp unit <us> */
  uint64_t available;
  LidarType type; /**<lidar type */

  uint32_t width;                          /**<the cols num */
  uint32_t height;                         /**<the rows num */
  uint32_t size;                           /**<actual used size */
  std::array<PointXYZIRTL, 300000> points; /**<the points ,maxnum is 1000000*/
  uint32_t index;

  enum : uint64_t {
    POINT_CLOUD_TYPE_ARRAY_SENSOR_NAME = 1 << 0,
    POINT_CLOUD_TYPE_ARRAY_SENSOR_TYPE = 1 << 1,
    POINT_CLOUD_TYPE_ARRAY_IS_END_TIME = 1 << 2,
    POINT_CLOUD_TYPE_ARRAY_WIDTH = 1 << 3,
    POINT_CLOUD_TYPE_ARRAY_HEIGHT = 1 << 4,
    POINT_CLOUD_TYPE_ARRAY_SIZE = 1 << 5,
    POINT_CLOUD_TYPE_ARRAY_POINTS = 1 << 6,

  };
};

struct HalfPointXYZIRTL {
  int16_t x;                     /**<x position */
  int16_t y;                     /**<y position */
  int16_t z;                     /**<z position */
  uint8_t intensity;             /**<intensity */
  uint16_t ring;                 /**<ring, laser */
  uint16_t timestamp_2us;        /**<timestamp unit 2*<us> */
  PcdPointLabel pcd_point_label; /**<point label, sky, ground, not roi*/
};

/**
 * @brief point cloud frame
 *
 */
struct PointCloudFrame {
  ad_std::Header header;
  PointCloudMeta meta;
  PointCloudTypeArray points;
};

/**
 * @brief half point cloud type array
 *
 */
struct HalfPointCloudTypeArray {
  int64_t timestamp; /**<timestamp unit <us> */
  uint64_t available;
  LidarType type; /**<lidar type */

  uint32_t width;    /**<the cols num */
  uint32_t height;   /**<the rows num */
  uint8_t size_half; /**<actual used size of points half */
  std::array<HalfPointXYZIRTL, 1000000> points_half; /**<the points */

  enum : uint64_t {
    POINT_CLOUD_TYPE_ARRAY_SENSOR_NAME = 1 << 0,
    POINT_CLOUD_TYPE_ARRAY_SENSOR_TYPE = 1 << 1,
    POINT_CLOUD_TYPE_ARRAY_IS_END_TIME = 1 << 2,
    POINT_CLOUD_TYPE_ARRAY_WIDTH = 1 << 3,
    POINT_CLOUD_TYPE_ARRAY_HEIGHT = 1 << 4,
    POINT_CLOUD_TYPE_ARRAY_SIZE_HALF = 1 << 5,
    POINT_CLOUD_TYPE_ARRAY_POINTS_HALF = 1 << 6,

  };
};

/**
 * @brief half point cloud frame
 *
 */
struct HalfPointCloudFrame {
  ad_std::Header header;
  PointCloudMeta meta;
  HalfPointCloudTypeArray points;
};

//************************************

enum class RadarDynamicProperty : uint8_t {
  DELETED = 0,
  DYNAMIC,
  STATIC,
  STOPPEDMOVING,
  UNKNOW,
};

enum class MeasureState : uint8_t {
  DELETED = 0,
  NEW,
  MEASURED,
  PREDICTED,
  UNKNOW
};

enum class RadarObstacleType : uint8_t {
  UNKNOWN = 0,
  POINT,
  CAR,
  TRUCK,
  PEDESTRIAN,
  MOTORCYCLE,
  BICYCLE,
  WIDE,
};

enum class RadarObjectOrigin : uint8_t {
  FRONT = 0,
  LEFTFRONT,
  RIGHTFRONT,
  LEFTREAR,
  RIGHTREAR
};

/**
 * @brief radar meta
 *
 */
struct RadarMeta {
  int64_t sensor_timestamp_us;
  int64_t start_timestamp_us;
  int64_t finish_timestamp_us;
};

/**
 * @brief radar object
 *
 */
struct RadarObject {
  ad_std::HeaderPOD header;
  RadarMeta meta;
  uint64_t available;          /**<binary,1 for available,0 for unavailable */
  uint32_t id;                 /**<the object track id */
  ad_std::Point2d position;  /**<the radar object position unit <m>*/
  ad_std::Vector2d velocity; /**<the radar object velocity unit<m/s>*/
  ad_std::Vector2d acceleration; /**<the radar object acceleration unit<m/s^2>*/
  ad_std::Vector2d acceleration_std; /**<the radar object acceleration std(ax,ax),(ay,ay)**/
  RadarDynamicProperty
      dynamic_property; /**<dynamic and static properties of the radar object */
  ad_std::CovarianceMatrix<double, 4> covariance; /**< the covariance include
   x-(0,0) y-(1,1) vx-(2,2) vy-(3,3)*/
  double age; /**<the radar object live age <s>*/
  double length; /**<the radar object length <m>*/
  double width;  /**<the radar object width <m>*/
  double yaw_angle;    /**<the radar object yaw angle*/
  double yaw_std;/**<the radar object yaw angle std*/
  double
      existence_probability;  /**<the radar obstacle probability of existence*/
  double radar_cross_section; /**<the radar obstacle radar cross-section
                                 unit<dBsm>*/
  MeasureState measure_state; /**<the radar obstacle measure state*/
  RadarObstacleType radar_obstacle_type; /**<the radar obstacle class*/
  enum : uint64_t {
    RADAR_OBJECT_ID = 1 << 0,
    RADAR_OBJECT_POSITION = 1 << 1,
    RADAR_OBJECT_VELOCITY = 1 << 2,
    RADAR_OBJECT_DYNAMIC_PROPERTY = 1 << 3,
    RADAR_OBJECT_COVARIANCE = 1 << 4,
    RADAR_OBJECT_EXISTENCE_PROBABILITY = 1 << 5,
    RADAR_OBJECT_RADAR_CROSS_SECTION = 1 << 6,
    RADAR_OBJECT_MEASURE_STATE = 1 << 7,
    RADAR_OBJECT_RADAR_OBSTACLE_CLASS = 1 << 8,
  };
};

/**
 * @brief the radar object array
 *
 */
struct RadarObjectArray : public ad_std::MessageBase  {
  ad_std::HeaderPOD header;
  RadarMeta meta;
  uint8_t size; ///< actual used  size
  RadarObject radar_objects[SENSOR_RADAR_OBJS_SIZE]; /**<the radar objects have no more than 100 */
  int radar_objects_size;
};

//************************************
struct UssMeta {
  int64_t sensor_timestamp_us;
  int64_t start_timestamp_us;
  int64_t finish_timestamp_us;
};

enum class UssStatus : uint8_t {
  BLOCKAGE_ERROR = 0x1,
  NOISE_ERROR = 0x2,
  HW_ERROR = 0x4,
  COMMUNICATION_ERROR = 0x8,
  PROXIMITY_ERROR = 0x10,
};

struct LongUssData {
  uint32_t id;          ///< the long wavelength uss instance ID
  uint32_t distance;    ///< the distance
  uint32_t width;       ///< the width
  UssStatus uss_status; ///< the uss status
  uint32_t reserved;
};

struct ShortUssData {
  uint32_t id;          ///< the short wavelength uss instance ID
  uint32_t distance;    ///< the distance
  UssStatus uss_status; ///< the uss status;
  uint32_t reserved;
};

struct UssData : public ad_std::MessageBase {
  ad_std::HeaderPOD header;
  UssMeta meta;
  uint64_t available;
  uint8_t long_uss_size; ///< actual used size
  LongUssData long_uss_data[SENSOR_USSDATA_LONG_SIZE];
  uint8_t short_uss_size; ///< actual used size
  ShortUssData short_uss_data[SENSOR_USSDATA_SHORT_SIZE];

  enum : uint64_t {
    USS_DATA_LONG_USS_SIZE = 1 << 0,
    USS_DATA_LONG_USS_DATA = 1 << 1,
    USS_DATA_SHORT_USS_SIZE = 1 << 2,
    USS_DATA_SHORT_USS_DATA = 1 << 3,

  };
};
//************************************

struct CanbusMeta {
  int64_t start_timestamp_us;
  int64_t finish_timestamp_us;
  /// timestamp of vehiclespeed,unit{us}
  int64_t timestamp_vehicle_speed_us;
  /// timestamp of yaw rate,unit{us}
  int64_t timestamp_yawrate_us;
  /// timestamp of rear left wheeelspeed,unit{us}
  int64_t timestamp_wheeelspeed_rl_us;
  /// timestamp of rear right wheeelspeed,unit{us}
  int64_t timestamp_wheelspeed_rr_us;
  /// timestamp of front left wheeelspeed,unit{us}
  int64_t timestamp_wheelspeed_fl_us;
  /// timestamp of front rear left  wheeelspeed,unit{us}
  int64_t timestamp_wheelspeed_fr_us;
  int64_t timestamp_wheelpulse_fl_us;
  int64_t timestamp_wheelpulse_fr_us;
  int64_t timestamp_wheelpulse_rl_us;
  int64_t timestamp_wheelpulse_rr_us;
};

/**
 * @brief DrivingDirection
 *  indicates driving direction of the vehicle
 */
enum class DrivingDirection : uint8_t {
  STOP = 0,      ///< STOP
  FORWARD,       ///< FORWARD
  BACKWARD,      ///< BACKWARD
  INVALID_VALUE, ///< INVALID VALUE
};

/**
 * @brief EPBSwitch
 *  indicates the epb switch is pressed or not.
 */
enum class EPBSwitch : uint8_t {
  NO_PRESS = 0, ///< NO PRESS
  UP,           ///< UP
  DOWN,         ///< DOWN
  RESERVED,     ///< RESERVED
};

/**
 * @brief SystemControlState
 * indicates status of system control
 */
enum class SystemControlState : uint8_t {
  INACTIVE = 0,     ///< INACTIVE OF SYSTEM CONTROL STATUS
  SVB_SVB1_ENABLE,  ///< SVB1 ENABLE OF SYSTEM CONTROL STATUS
  ADAS_SVB1_ENABLE, ///< ADAS AND SVB1 ENABLE OF SYSTEM CONTROL STATUS
  SVB_SVB2_ENABLE,  ///< SVB2 ENABLE OF SYSTEM CONTROL STATUS
};

/// INDICATES STATUS OF BRAKE AUTO CONTROL
enum class BrakingAutoControlState : uint8_t {
  INACTIVE = 0, ///< INACTIVE OF BRAKE AUTO CONTROL
  ACTIVE,       ///< ACTIVE OF BRAKE AUTO CONTROL
  DEGRADE,      ///< DEGRADE OF BRAKE AUTO CONTROL
  RESERVED,     ///< RESERVE OF BRAKE AUTO CONTROL
};

/// INDICATES STATUS OF DRIVING AUTO CONTROL
enum class DrivingAutoControlState : uint8_t {
  INACTIVE = 0, ///< INACTIVE OF DRIVING AUTO CONTROL STATUS
  ACTIVE,       ///< ACTIVE OF DRIVING AUTO CONTROL STATUS
  DEGRADE,      ///< DEGRADE OF DRIVING AUTO CONTROL STATUS
  RESERVED,     ///< RESERVE OF DRIVING AUTO CONTROL STATUS
};

/// INDICATES STATUS OF STEERING AUTO CONTROL
enum class SteeringAutoControlState : uint8_t {
  INACTIVE = 0, ///< INACTIVE OF STEERING AUTO CONTROL STATUS
  ACTIVE,       ///< ACTIVE OF STEERING AUTO CONTROL STATUS
  DEGRADE,      ///< DEGRADE OF STEERING AUTO CONTROL STATUS
  RESERVED,     ///< RESERVE OF STEERING AUTO CONTROL STATUS
};

/// INDICATES STATUS OF SYSTEM FAILURE
enum class SystemFailureState : uint8_t {
  /// NOT FAIL OF SYSTEM FAILURE STATUS
  NOT_FAIL = 0,
  /// LOSE SVB_SVB1 OF SYSTEM FAILURE STATUS
  LOSE_SVB_SVB1,
  /// LOSE ADAS_SVB1 OF SYSTEM FAILURE STATUS
  LOSE_ADAS_SVB1,
  /// LOSE SVB_SVB2 OF SYSTEM FAILURE STATUS
  LOSE_SVB_SVB2,
  /// LOSE SVB_SVB1 AND ADAS_SVB1 OF SYSTEM FAILURE STATUS
  LOSE_SVB_SVB1_ADAS_SVB1,
  /// LOSE SVB_SVB1 AND SVB_SVB2 OF SYSTEM FAILURE STATUS
  LOSE_SVB_SVB1_SVB_SVB2,
  /// LOSE ADAS_SVB1 AND SVB_SVB2 OF SYSTEM FAILURE STATUS
  LOSE_ADAS_SVB1_SVB_SVB2,
  /// LOSE SVB_SVB1 AND ADAS_SVB1 AND SVB_SVB2 OF SYSTEM FAILURE STATUS
  LOSE_SVB_SVB1_ADAS_SVB1_SVB_SVB2,
};

/// INDICATES STATUS OF BRAKING FAILURE
enum class BrakingFailureState : uint8_t {
  /// NOT FAIL OF BRAKING FAILURE STATUS
  NOT_FAIL = 0,
  /// EPB ERROR OF BRAKING FAILURE STATUS
  EPB_ERR,
  /// CDDS_AEB ERROR OF BRAKING FAILURE STATUS
  CDDS_AEB_ERR,
  /// VLC_VMC ERROR OF BRAKING FAILURE STATUS
  VLC_VMC_ERR,
  /// EPB AND CDDS_AEB ERROR OF BRAKING FAILURE STATUS
  EPB_CDDS_AEB_ERR,
  /// EPB AND VLC_VMC ERROR OF BRAKING FAILURE STATUS
  EPB_VLC_VMC_ERR,
  /// CDDS_AEB AND VLC_VMC ERROR OF BRAKING FAILURE STATUS
  CDDS_AEB_VLC_VMC_ERR,
  /// EPB AND CDDS_AEB AND VLC_VMC ERROR OF BRAKING FAILURE STATUS
  EPB_CDDS_AEB_VLC_VMC_ERR,
};

/// INDICATES STATUS OF DRIVING FAILURE
enum class DrivingFailureState : uint8_t {
  NOT_FAIL = 0,    ///< NOT FAIL OF DRIVING FAILURE STATUS
  YRS_ERR,         ///< YRS ERROR OF DRIVING FAILURE STATUS
  Spd_ERR,         ///< SPD ERROR OF DRIVING FAILURE STATUS
  HCU_ERR,         ///< HCU ERROR OF DRIVING FAILURE STATUS
  YRS_Spd_ERR,     ///< YRS AND SPEED ERROR OF DRIVING FAILURE STATUS
  YRS_HCU_ERR,     ///< YRS AND HCU ERROR OF DRIVING FAILURE STATUS
  Spd_HCU_ERR,     ///< SPEED AND HCU ERROR OF DRIVING FAILURE STATUS
  YRS_Spd_HCU_ERR, ///< YRS AND HCU ERROR OF DRIVING FAILURE STATUS
};

/// INDICATES STATUS OF STEERING FAILURE
enum class SteeringFailureState : uint8_t {
  /// NOT FAIL OF STEERING FAILURE STATUS
  NOT_FAIL = 0,
  /// SAS ERROR OF STEERING FAILURE STATUS
  SAS_ERR,
  /// EPS ERROR OF STEERING FAILURE STATUS
  EPS_ERR,
  /// APA ERROR OF STEERING FAILURE STATUS
  APA_ERR,
  /// TJP ERROR OF STEERING FAILURE STATUS
  TJP_ERR,
  /// SAS AND EPS ERROR OF STEERING FAILURE STATUS
  SAS_EPS_ERR,
  /// SAS AND APA ERROR OF STEERING FAILURE STATUS
  SAS_APA_ERR,
  /// SAS AND TJP ERROR OF STEERING FAILURE STATUS
  SAS_TJP_ERR,
  /// EPS AND APA ERROR OF STEERING FAILURE STATUS
  EPS_APA_ERR,
  /// EPS AND TJP ERROR OF STEERING FAILURE STATUS
  EPS_TJP_ERR,
  /// APA AND TJP ERROR OF STEERING FAILURE STATUS
  APA_TJP_ERR,
  /// SAS AND EPS AND APA ERROR OF STEERING FAILURE STATUS
  SAS_EPS_APA_ERR,
  /// SAS AND EPS AND TJP ERROR OF STEERING FAILURE STATUS
  SAS_EPS_TJP_ERR,
  /// SAS AND APA AND TJP ERROR OF STEERING FAILURE STATUS
  SAS_APA_TJP_ERR,
  /// EPS AND APA TJP ERROR OF STEERING FAILURE STATUS
  EPS_APA_TJP_ERR,
  /// SAS AND EPS AND APA AND TJP ERROR OF STEERING FAILURE STATUS
  SAS_EPS_APA_TJP_ERR,
};

/// INDICARES EPB READY OR NOT
enum class EPBReadyState : uint8_t {
  /// READY OF EPB STATUS
  READY = 0,
  /// REQ UNREADY OF EPB STATUS
  REQUNREADY,
  /// SEATBELT OR DOOR UNREADY OF EPB STATUS
  SEATBELTORDOORUNREADY,
  /// L2 AND L3 UNREADY OF EPB STATUS
  L2ANDL3UNREADY,
  /// REQ_SEATBELT OR DOOR UNREADY OF EPB STATUS
  REQ_SEATBELTORDOORUNREADY,
  /// REQ_L2 AND L3 UNREADY OF EPB STATUS
  REQ_L2ANDL3UNREADY,
  /// SEATBELT OR DOOR_L2 AND L3 UNREADY OF EPB STATUS
  SEATBELTORDOOR_L2ANDL3UNREADY,
  /// All UNREADY OF EPB STATUS
  AllUNREADY,
};

/// INDICATES THE STATUS OF SAS
enum class SasState : uint8_t {
  SAS_ANGLE_AND_SPEED_CORRECT = 0, ///< SAS ANGLE AND SPEED CORRECT
  SAS_NOT_CALIBRATE,               ///< SAS NOT CALIBRATE
  INTERMITTENT_ERROR_DETECTED,     ///< INTERMITTENT ERROR DETECTED
  PERMANENT_ERROR_DETECTED,        ///< PERMANENT ERROR DETECTED
};

/// INDICATES ERROR OF EPS SYSTEM
enum class ErrorState : uint8_t {
  NO_ERROR = 0,          ///< NO ERROR OF EPS SYSYTEM
  GENERAL_ERROR,         ///< GENERAL ERROR OF EPS SYSTEM
  CRITICAL_ERROR,        ///< CRITICAL ERROR OF EPS SYSTEM
  ERROR_STATUS_RESERVED, ///< RESERVE OF EPS SYSTEM
};

/// WORKING STATSUS OF EPB
enum class EPBWorkingState : uint8_t {
  UNKNOW = 0,
  RELEASE,              ///< RELEASE OF EPB
  RELEASING_OR_LOCKING, ///< RELEASING OR LOCKING OF EPB
  EPB_LOCKED,           ///< LOCKED OF EPB
};

/// STATSUS OF BRAKE PEDAL
enum class BrakePedalStateQ : uint8_t {
  NOT_INITIALIZED = 0,  ///< NOT INITIALIZED OF BRAKE PEDAL
  FAULT,                ///< FAULT OF BRAKE PEDAL
  FULL,                 ///< FULL OF BRAKE PEDAL
  BRAKE_PEDAL_RESERVED, ///< RESERVE OF BRAKE PEDAL
};

/// GEAR INFORMATION
enum class GearInfo : uint8_t {
  P = 0,                   ///< P GEAR
  R,                       ///< R GEAR
  N,                       ///< N GEAR
  D,                       ///< D GEAR
  GEAR_INFO_Reserved1,     ///< RESERVED1
  NO_CONNECTION,           ///< NO CONNECTION OF GEAR
  GEAR_INFO_RESERVED2,     ///< RESERVED2
  GEAR_INFO_INVALID_VALUE, ///< INVALID_VALUE OF GEAR
};

/// SEATBELT STATUS OF DRIVER
enum class DriverSeatBeltState : uint8_t {
  SEATBELT_DRIVER_UNBUCKLED = 0, ///< UNBUCKLED OF SEATBELT
  BUCKLED,                       ///< BUCKLED OF SEATBELT
  SEATBELT_DRIVER_RESERCED1,     ///< RESERCED1 OF SEATBELT
  SEATBELT_DRIVER_RESERVED2,     ///< RESERVED2 OF SEATBELT
};

/// WIPER STATUS OF FRONT
enum class FrontWiperState : uint8_t {
  OFF = 0,                  ///< FRONT WIPER STATUS IS OFF
  TIP,                      ///< FRONT WIPER STATUS IS TIP
  INTERVAL_OR_AUTO_LEVER,   ///< FRONT WIPER STATUS IS INTERVAL OR AUTO LEVER
  LOW_SPEED,                ///< FRONT WIPER STATUS IS LOW SPEED
  HIGH_SPEED,               ///< FRONT WIPER STATUS IS HIGH SPEED
  FRONT_WIPER_ST_RESERVED1, ///< FRONT WIPER STATUS IS RESERVED1
  FRONT_WIPER_ST_RESERVED2, ///< FRONT WIPER STATUS IS RESERVED2
  FRONT_WIPER_ST_RESERVED3, ///< FRONT WIPER STATUS IS RESERVED3
};

/// THE SPEED OF FRONT WIPER
enum class FrontWiperSpeed : uint8_t {
  NO_WIPER = 0,    ///< NO WIPE OF FRONT WIPER
  TIMES_PER_MIN42, ///< 42 TIMES PER MINUTE OF FRONT WIPER
  TIMES_PER_MIN45, ///< 45 TIMES PER MINUTE OF FRONT WIPER
  TIMES_PER_MIN48, ///< 48 TIMES PER MINUTE OF FRONT WIPER
  TIMES_PER_MIN51, ///< 51 TIMES PER MINUTE OF FRONT WIPER
  TIMES_PER_MIN54, ///< 54 TIMES PER MINUTE OF FRONT WIPER
  TIMES_PER_MIN57, ///< 57 TIMES PER MINUTE OF FRONT WIPER
  TIMES_PER_MIN60, ///< 60 TIMES PER MINUTE OF FRONT WIPER
};

/// THE STATUS OF SUNSHADE
enum class SunShadeState : uint8_t {
  /// FULL CLOSED OF SUNSHADE
  SUBSHADE_FULLY_CLOSED = 0,
  /// BETWEEM FULL CLOSED AND PARALLEL OPEN LIMIT OF SUNSHADE
  SUBSHADE_BETWEEM_FULL_CLOSED_AND_PARALLEL_OPEN_LIMIT,
  /// PARALLEL OPEN LIMIT OF SUNSHADE
  SUBSHADE_PARALLEL_OPEN_LIMIT,
  /// INVALID VALUE OF SUNSHADE
  SUBSHADE_INVALID_VALUE,
};

/// THE STATUS OF SUNROOF
enum class SunRoofState : uint8_t {
  /// FULL CLOSED OF SUNROOF
  SUNROOF_FULLY_CLOSED = 0,
  /// BETWEEN FULLY CLOSED AND TILT OPEN LIMIT OF SUNROOF
  SUNROOF_BETWEEN_FULLY_CLOSED_AND_TILT_OPEN_LIMIT,
  /// TILT OPEN LIMIT IF SUNROOF
  SUNROOF_TILT_OPEN_LIMIT,
  /// BETWEEM FULLY CLOSED AND PARALLEL OPEN LIMIT OF SUNROOF
  SUNROOF_BETWEEM_FULLY_CLOSED_AND_PARALLEL_OPEN_LIMIT,
  /// PARALLEL OPEN LIMIT OF SUNROOF
  SUNROOF_PARALLEL_OPEN_LIMIT,
  /// PARALLEL OPEN HALF OF SUNROOF
  SUNROOF_PARALLEL_OPEN_HALF,
  /// RESERVED OF SUNROOF
  SUNROOF_RESERVED,
  /// INVALID VALUE OF SUNROOF
  SUNROOF_INVALID_VALUE,
};

/// FRONT LEFT DOOR STATUS
enum class DoorLockState : uint8_t {
  LOCK = 0,  ///< THE STATUS OF FRONT LEFT DOOR IS LOCK
  UNLOCK,    ///< THE STATUS OF FRONT LEFT DOOR IS UNLOCK
  RESERVED1, ///< RESERVED1
  RESERVED2, ///< RESERVED2
};

///< THE STATUS OF FRONT LEFT DOOR WINDOW
enum class DoorWindowState : uint8_t {
  /// THE STATUS OF FRONT LEFT DOOR WINDOW IS UNKNOWN
  UNKNOWN = 0,
  /// THE STATUS OF FRONT LEFT DOOR WINDOW IS TOP
  TOP,
  /// THE STATUS OF FRONT LEFT DOOR WINDOW IS MIDDELE
  MIDDELE,
  /// THE STATUS OF FRONT LEFT DOOR WINDOW IS BOTTOM
  BOTTOM,
  /// THE STATUS OF FRONT LEFT DOOR WINDOW IS VENTILATION
  VENTILATION,
  /// OTHER
  OTHER,
  /// RESERVED1
  RESERVED1,
  /// RESERVED2
  RESERVED2,
};

enum class DrivingMode : uint8_t {
  COMPLETE_MANUAL = 0, ///< HUMAN MANUAL
  COMPLETE_AUTO_MODE,  ///< AUTO DRIVE
  AUTO_STEER_ONLY,     ///< ONLY STEERING ENABLE
  AUTO_SPEED_ONLY,     ///< ONLY SPEED ENABLE
  EMERGENCY_MODE,      ///< EMERGENCY MODE
};

struct WheelSpeedInfo {
  uint64_t avaliable;
  double rl; ///< THE WHEEL SPEED OF REAR LEFT
  double rr; ///< THE WHEEL SPEED OF REAR RIGHT
  double fl; ///< THE WHEEL SPEED OF FRONT LEFT
  double fr; ///< THE WHEEL SPEED OF FRONT RIGHT
  enum : uint64_t {
    RL = 1 << 0,
    RR = 1 << 1,
    FL = 1 << 2,
    FR = 1 << 3,
  };
};
struct WheelPulseInfo {
  uint64_t avaliable;
  double rl; ///< THE WHEEL PULSE OF REAR LEFT
  double rr; ///< THE WHEEL PULSE OF REAR RIGHT
  double fl; ///< THE WHEEL PULSE OF FRONT LEFT
  double fr; ///< THE WHEEL PULSE OF FRONT RIGHT
  enum : uint64_t {
    RL = 1 << 0,
    RR = 1 << 1,
    FL = 1 << 2,
    FR = 1 << 3,
  };
};

/// INDICATES THE AUTO CONTROL STATUS
struct AutoControlState {
  uint64_t avaliable;
  SystemControlState
      system_control_state; ///< INDICATES STATUS OF SYSTEM CONTROL
  BrakingAutoControlState
      braking_auto_control_state; ///< INDICATES STATUS OF BRAKE AUTO CONTROL
  DrivingAutoControlState
      driving_auto_control_state; ///< INDICATES STATUS OF DRIVING AUTO CONTROL
  SteeringAutoControlState
      steering_auto_control_state; ///< INDICATES STATUS OF STEERING AUTO
                                   ///< CONTROL
  enum : uint64_t {
    SYSTEM_CONTROL_STATE = 1 << 0,
    BRAKING_AUTO_CONTROL_STATE = 1 << 1,
    DRIVING_AUTO_CONTROL_STATE = 1 << 2,
    STEERING_AUTO_CONTROL_STATE = 1 << 3,
  };
};

/// indicates the auto control status including system_failure braking_failure
/// driving_failure and steering_failure
struct VehicleFailureState {
  uint64_t avaliable;
  SystemFailureState
      system_failure_state; ///< INDICATES STATUS OF SYSTEM FAILURE
  BrakingFailureState
      braking_failure_state; ///< INDICATES STATUS OF BRAKING FAILURE
  DrivingFailureState
      driving_failure_state; ///< INDICATES STATUS OF DRIVING FAILURE
  SteeringFailureState
      steering_failure_state; ///< INDICATES STATUS OF STEERING FAILURE
  enum : uint64_t {
    SYSTEM_FAILURE_STATE = 1 << 0,
    BRAKING_FAILURE_STATE = 1 << 1,
    DRIVING_FAILURE_STATE = 1 << 2,
    STEERING_FAILURE_STATE = 1 << 3,
  };
};

/// indicates the takeover status including braking_takeover dirving_takeover
/// and steering_takeover
struct TakeoverState {
  uint64_t avaliable;
  bool braking;  ///< BRAKE TAKEOVER STATUS
  bool driving;  ///< DRIVING TAKEOVER STATUS
  bool steering; ///< STEERING TAKEOVER STATUS
  enum : uint64_t {
    BRAKING = 1 << 0,
    DRIVING = 1 << 1,
    STEERING = 1 << 2,
  };
};

/// indicates the steering wheel info
struct SteeringWheelInfo {
  uint64_t avaliable;
  double angle;    ///< THE ANGLE OF STEERING WHEEL
  bool angle_sign; ///< THE ANGLE SIGN OF STEERING WHEEL
  double speed;    ///< THE SPEED OF STEERING WHEEL
  bool speed_sign; ///< THE SPEED SIGN OF STEERING WHEEL
  enum : uint64_t {
    ANGLE = 1 << 0,
    ANGLE_SIGN = 1 << 1,
    SPPED = 1 << 2,
    SPEED_SIGN = 1 << 3,
  };
};

/// indicates the switch
struct ACCSwitch {
  uint64_t avaliable;
  bool cruise_main_switch;              ///< CRUISE MAIN SWITCH
  bool cruise_cancel_switch;            ///< CRUISE CANCEL SWITCH
  bool set_or_decrease_speed_switch;    ///< SET OR DECREASE SPEED SWITCH
  bool resume_or_increase_speed_switch; ///< SET OR INCREASE SPEED SWITCH
  bool increase_distance_switch;        ///< INCREASE DISTANCE SWITCH
  bool decrease_distance_switch;        ///< DECREASE DISTANCE SWITCH
  enum : uint64_t {
    CRUISE_MAIN_SWITCH = 1 << 0,
    CRUISE_CANCEL_SWITCH = 1 << 1,
    SET_OR_DECREASE_SPEED_SWITCH = 1 << 2,
    RESUME_OR_INCREASE_SPEED_SWITCH = 1 << 3,
    INCREASE_DISTANCE_SWITCH = 1 << 4,
    DECREASE_DISTANCE_SWITCH = 1 << 5,
  };
};

/// indicates the light status of vehicle
struct LightState {
  uint64_t avaliable;
  bool position_light_state; ///< THE STATUS OF LIGHT
  bool low_beam_state;       ///< THE STATUS OF LOW BEAM
  bool high_beam_state;      ///< THE STATUS OF HIGH BEAM
  bool turning_state_left;   ///< THE STATUS OF TURNNING LEFT
  bool turning_state_right;  ///< THE STATUS OF TURNING RIGHT
  bool hazard_warning_state; ///< THE STATUS OF HAZARD WARNING STATUS
  bool horn;                 ///< THE STATUS OF HORN
  uint8_t turning_state;     ///< THE STATUS OF TURNING
  enum : uint64_t {
    POSITION_LIGHT_STATE = 1 << 0,
    LOW_BEAM_STATE = 1 << 1,
    HIGH_BEAM_STATE = 1 << 2,
    TURNING_STATE_LEFT = 1 << 3,
    TURNING_STATE_RIGHT = 1 << 4,
    HAZARD_WARNING_STATE = 1 << 5,
    TURNING_STATE = 1 << 6,
  };
};

/// door status
/// ture represent open,false represent close.
struct DoorState {
  uint64_t avaliable;
  /// the door status of front right
  DoorLockState fr;
  bool open_state_fr;
  /// the door status of front left
  DoorLockState fl;
  bool open_state_fl;
  /// the door status of rear left
  DoorLockState rl;
  bool open_state_rl;
  /// the door status of rear right
  DoorLockState rr;
  bool open_state_rr;
  /// the status of luggage door
  bool luggage;
  enum : uint64_t {
    FR = 1 << 0,
    OPEN_STATE_FR = 1 << 1,
    FL = 1 << 2,
    OPEN_STATE_FL = 1 << 3,
    RL = 1 << 4,
    OPEN_STATE_RL = 1 << 5,
    RR = 1 << 6,
    OPEN_STATE_RR = 1 << 7,
    LUGGAGE = 1 << 8,
  };
};

/// the status of window
struct WindowState {
  uint64_t avaliable;
  DoorWindowState fl; ///< THE STATUS OF FRONT LEFT DOOR WINDOW
  DoorWindowState fr; ///< THE STATSUS OF FRONT RIGHT DOOR WINDOW
  DoorWindowState rl; ///< THE STATUS OF REAR LEFT DOOR WINDOWN
  DoorWindowState rr; ///< THE STATUS OF REAR RIGHT DOOR WINDOW
  enum : uint64_t {
    FL = 1 << 0,
    FR = 1 << 1,
    RL = 1 << 2,
    RR = 1 << 3,
  };
};
/// indicate epb signal
struct EPB {
  uint64_t avaliable;
  EPBSwitch epb_switch; ///< INDICATES THE EPB SWITCH IS PRESSED OR NOT.
  EPBReadyState epb_ready_state;     ///< INDICARES EPB READY OR NOT
  EPBWorkingState epb_working_state; ///< WORKING STATSUS OF EPB
  enum : uint64_t {
    EPB_SWITCH = 1 << 0,
    EPB_READY_STATE = 1 << 1,
    EPB_WORKING_STATE = 1 << 2,
  };
};
/// indicates lateral signal
struct EPS {
  uint64_t avaliable;
  SteeringWheelInfo steering_wheel_info; ///< INDICATES THE STEERING WHEEL INFO
  SasState sas_state;                    ///< INDICATES THE STATUS OF SAS
  double hand_steering_torque;           ///< HAND STEERING TORQUE
  bool hand_steering_torque_sign;        ///< HAND STEERING TORQUE SIGN
  enum : uint64_t {
    STEERING_WHEEL_INFO = 1 << 0,
    SAS_STATE = 1 << 1,
    HAND_STEERING_TORQUE = 1 << 2,
    HAND_STEERING_TORQUE_SIGN = 1 << 3,
  };
};

/// indicate lon signal
struct ESP {
  uint64_t avaliable;
  DrivingDirection
      driving_direction; ///< INDICATES DRIVING DIRECTION OF THE VEHICLE
  WheelSpeedInfo wheel_speed_info;         ///< INDICATES THE WHEEL SPPED
  WheelPulseInfo wheel_pulse_info;         ///< INDICATES THE WHEEL PULSE
  double master_cylinder_pressure;         ///< MASTER CYLINDER PRESSURE
  double vehicle_speed;                    ///< VEHICLE SPEED
  bool vehicle_speed_q;                    ///< VEHICLE SPPED Q
  double acceleration_pedal_position;      ///< ACCELERACTION PEDAL POSITION
  double calc_acceleration_pedal_position; ///< THE CALCULATION OF ACCELERATION
  bool brake_pedal_state;                  ///< THE STATUS OF BRAKE PEADA
  BrakePedalStateQ brake_pedal_state_q;    ///< STATSUS OF BRAKE PEDAL
  enum : uint64_t {
    DRIVING_DIRECTION = 1 << 0,
    WHEEL_SPEED_INFO = 1 << 1,
    WHEEL_PULSE_INFO = 1 << 2,
    MASTER_CYLINDER_PRESSURE = 1 << 3,
    VEHICLE_SPEED = 1 << 4,
    VEHICLE_SPEED_Q = 1 << 5,
    ACCELERATION_PEDAL_POSITION = 1 << 6,
    CALC_ACCELERATION_PEDAL_POSITION = 1 << 7,
    BRAKE_PEDAL_STATE = 1 << 8,
    BRAKE_PEDAL_STATE_Q = 1 << 9,
  };
};
/// indicate yaw rate sensor info
struct YRS {
  uint64_t avaliable;
  double yaw_rate;       ///< YAW RATE
  bool yrs_state;        ///< THE STATUS OF TRS
  double acceleration_x; ///< THE ACCELERATION OF X
  double yaw_rate_raw;   ///< YAW RATE RAW
  double acceleration_y; ///< THE ACCELERATION OF Y
  bool error_state_yrs;  ///< THE ERROR STATUS OF YRS
  enum : uint64_t {
    YAW_RATE = 1 << 0,
    YRS_STATE = 1 << 1,
    ACCELERATION_X = 1 << 2,
    YAW_RATE_RAW = 1 << 3,
    ACCELERATION_Y = 1 << 4,
    ERROR_STATE_YRS = 1 << 5,
  };
};

struct ChassisInfo {
  uint64_t avaliable;
  EPB epb;
  EPS eps;
  ESP esp;
  YRS yrs;
  ad_std::GearState gear_info; ///< GEAR INFORMATION
  enum : uint64_t {
    CHASSIS_INFO_EPB = 1 << 0,
    CHASSIS_INFO_EPS = 1 << 1,
    CHASSIS_INFO_ESP = 1 << 2,
    CHASSIS_INFO_YRS = 1 << 3,
    CHASSIS_INFO_gear_info = 1 << 4,
  };
};
struct BodyInfo {
  uint64_t avaliable;
  ACCSwitch acc_switch;                      ///< NDICATES THE ACCSWITCH
  DriverSeatBeltState driver_seatbelt_state; ///< SEATBELT STATUS OF DRIVER
  LightState light_state;            ///< INDICATES THE LIGHT STATUS OF VEHICLE
  FrontWiperState front_wiper_state; ///< WIPER STATUS OF FRONT
  FrontWiperSpeed front_wiper_speed; ///< THE SPEED OF FRONT WIPER
  SunShadeState sun_shade_state;     ///< THE STATUS OF SUNSHADE
  SunRoofState sun_roof_state;       ///< THE STATUS OF SUNROOF
  DoorState door_state;              ///< DOOR STATUS
  WindowState window_state;          ///< THE STATUS OF WINDOW
  int32_t battery_pack_soc;          ///< THE BATTERY OF PACK
  bool button1;                      ///< BUTTON1
  bool button2;                      ///< BUTTON2
  enum : uint64_t {
    ACCSWITCH = 1 << 0,
    DRIVER_SEATBELT_STATE = 1 << 1,
    LIGHT_STATE = 1 << 2,
    FRONT_WIPER_STATE = 1 << 3,
    FRONT_WIPER_SPEED = 1 << 4,
    SUN_SHADE_STATE = 1 << 5,
    SUN_ROOF_STATE = 1 << 6,
    DOOR_STATE = 1 << 7,
    WINDOW_STATE = 1 << 8,
    BATTERY_PACK_SOC = 1 << 9,
    BUTTON1 = 1 << 10,
    BUTTON2 = 1 << 11,
  };
};

struct Canbus : public ad_std::MessageBase {
  ad_std::HeaderPOD header;
  CanbusMeta meta;
  uint64_t available;
  ChassisInfo chassis_info; ///< INDICATES THE CHASSIS INFO OF VEHICLE
  BodyInfo body_info;       ///< INDICATES THE BODY INFO OF VEHICLE
  AutoControlState auto_control_state; ///< INDICATES THE AUTO CONTROL STATUS
  VehicleFailureState
      vehicle_failure_state;    ///< INDICATES THE AUTO CONTROL STATUS
  TakeoverState takeover_state; ///< INDICATES THE TAKEOVER STATUS
  ErrorState error_status;      ///< INDICATES ERROR OF EPS SYSTEM
  DrivingMode driving_mode;
  int32_t door_switch_status;   ///< INDICATES THE DOOR STATUS, 0 for close, 1 for open

  enum : uint64_t {
    CHASSIS_INFO = 1 << 0,
    BODY_INFO = 1 << 1,
    AUTO_CONTROL_STATE = 1 << 2,
    VEHICLE_FAILURE_STATE = 1 << 3,
    TAKEOVER_STATE = 1 << 4,
    ERROR_STATUS = 1 << 5,
    DRIVING_MODE = 1 << 6,
  };
};
//************************************

/**
 * @brief This is a message to hold data from an IMU (Inertial Measurement Unit)
 * Accelerations should be in m/s^2 (not in g's), and rotational velocity should
 *be in rad/sec If the covariance of the measurement is known, it should be
 *filled in (if all you know is the variance of each measurement, e.g. from the
 *datasheet, just put those along the diagonal) A covariance matrix of all zeros
 *will be interpreted as "covariance unknown", and to use the data a covariance
 *will have to be assumed or gotten from some other source If you have no
 *estimate for one of the data elements (e.g. your IMU doesn't produce
 *anorientation estimate), please set element 0 of the associated covariance
 *matrix to -1 If you are interpreting this message, please check for a value of
 *-1 in the first element of each covariance matrix, and disregard the
 *associated estimate.
 */

struct ImuMeta {
  int64_t start_timestamp_us;
  int64_t finish_timestamp_us;
};

struct Imu : public ad_std::MessageBase {
  ad_std::HeaderPOD header;
  ImuMeta meta;
  uint64_t available;
  ad_std::OrientationWithCovariance orientation_info;
  ad_std::LinearAccelerationWithCovariance linear_acceleration_info;
  ad_std::AngularVelocityWithCovariance angular_velocity_info;

  enum : uint64_t {
    IMU_ORIENTATION_INFO = 1 << 0,
    IMU_LINEAR_ACCELERATION_INFO = 1 << 1,
    IMU_ANGULAR_VELOCITY_INFO = 1 << 2,
  };
};
//************************************

/**
 * @brief This represents an estimate of a position and velocity in free space.
 * The pose in this message should be specified in the coordinate frame given by
 * header.frame_id.
 * The twist in this message should be specified in the coordinate frame given
 * by the child_frame_id
 */

namespace ad_ins {

struct OdomMeta {
  int64_t start_timestamp_us;
  int64_t finish_timestamp_us;
};

struct Odometry : public ad_std::MessageBase {
  ad_std::HeaderPOD header;
  OdomMeta meta;
  uint64_t available;
  char child_frame_id[SENSOR_ID_SIZE];
  ad_std::PoseWithCovariance pose_with_covariance;
  ad_std::TwistWithCovariance twist_with_covariance;

  enum : uint64_t {
    ODOMETRY_CHILD_FRAME_ID = 1 << 0,
    ODOMETRY_POSE_WITH_COVARIANCE = 1 << 1,
    ODOMETRY_TWIST_WITH_COVARIANCE = 1 << 2,
  };
};
//************************************
struct GpsFixMeta {
  int64_t start_timestamp_us;
  int64_t finish_timestamp_us;
};

enum class GpsStatusMeasurementStatus : int8_t {
  STATUS_NO_FIX = -1,   ///< Unable to fix position
  STATUS_FIX = 0,       ///< Normal fix
  STATUS_SBAS_FIX = 1,  ///< Fixed using a satellite-based augmentation system
  STATUS_GBAS_FIX = 2,  ///< or a ground-based augmentation system
  STATUS_DGPS_FIX = 18, ///< Fixed with DGPS
  STATUS_WAAS_FIX = 33, ///< Fixed with WAAS
};

enum class GpsStatusSource : uint8_t {
  SOURCE_NONE = 0, ///< No information is available
  SOURCE_GPS =
      1, ///< Using standard GPS location [only valid for position_source]
  SOURCE_POINTS =
      2, ///< Motion/orientation fix is derived from successive points
  SOURCE_DOPPLER = 4,   ///< Motion is derived using the Doppler effect
  SOURCE_ALTIMETER = 8, ///< Using an altimeter
  SOURCE_MAGNETIC = 16, ///< Using magnetic sensors
  SOURCE_GYRO = 32,     ///< Using gyroscopes
  SOURCE_ACCEL = 64,    ///< Using accelerometers
};

struct GpsStatus {
  ad_std::HeaderPOD header;
  uint16_t satellites_used;                        ///< Number of satellites
  std::vector<uint32_t> satellite_used_prn;        ///< PRN identifiers
  uint16_t satellites_visible;                     ///< Satellites visible
  std::vector<uint32_t> satellite_visible_prn;     ///< PRN identifiers
  std::vector<uint32_t> satellite_visible_z;       ///< Elevation of satellites
  std::vector<uint32_t> satellite_visible_azimuth; ///< Azimuth of satellites
  std::vector<uint32_t> satellite_visible_snr; ///< Signal-to-noise ratios (dB)

  GpsStatusMeasurementStatus measurement_status; ///< Measurement status
  GpsStatusSource motion_source;      ///< Source for speed, climb and track
  GpsStatusSource orientation_source; ///< Source for device orientation
  GpsStatusSource position_source;    ///< Source for position
};

struct GpsFixPosition {
  /// Latitude (degrees). Positive is north of equator;negative is south
  double latitude;
  /// Longitude (degrees). Positive is east of prime meridian, negative west
  double longitude;
  /// Altitude (meters). Positive is above reference (e.g., sea level).
  double altitude;
  /// Spherical position uncertainty (meters) [epe]
  double err;
  /// Horizontal position uncertainty (meters) [eph]
  double err_horz;
  /// Vertical position uncertainty (meters) [epv]
  double err_vert;
  /// Position covariance [m^2] defined relative to a tangential plane through
  /// the reported position.The components are East, North, and Up (ENU),
  /// inrow-major order
  ad_std::CovarianceMatrix<double, 9> position_covariance;
};

struct GpsFixTrack {
  double track;     ///< Direction (degrees from north) unit<rad>
  double err_track; ///< Track uncertainty (degrees) [epd]
};

struct GpsFixSpeed {
  double speed;     ///< speed (meters/second)
  double err_speed; ///< speed uncertainty (meters/second) [eps]
};

struct GpsFixOrientation {
  double pitch;     ///< Device orientation (units in degrees)
  double roll;      ///< Device orientation (units in degrees)
  double dip;       ///< Device orientation (units in degrees)
  double err_pitch; ///< Orientation uncertainty (degrees)
  double err_roll;  ///< Orientation uncertainty (degrees)
  double err_dip;   ///< Orientation uncertainty (degrees)
};

struct GpsFixGpsTime {
  double time;     ///< GPS time unit<s>
  double err_time; ///< Temporal uncertainty [ept]
};

struct GpsFixDop {
  double gdop; ///< Total (positional-temporal) dilution of precision
  double pdop; ///< Positional (3D) dilution of precision
  double hdop; ///< Horizontal dilution of precision
  double vdop; ///< Vertical dilution of precision
  double tdop; ///< Temporal dilution of precision
};

enum class PosCovType : uint8_t {
  UNKNOWN = 0,
  APPROXIMATED,
  DIAGONAL_KNOWN,
  KNOWN,
};

struct GpsFix {
  ad_std::HeaderPOD header;
  GpsFixMeta meta;
  uint64_t available;
  GpsStatus gps_status;
  GpsFixPosition position;
  GpsFixTrack track;
  GpsFixSpeed ground_speed;
  GpsFixSpeed vertical_speed;
  GpsFixOrientation orientation;
  GpsFixGpsTime gps_time;
  GpsFixDop dop;
  PosCovType pos_cov_type;

  enum : uint64_t {
    GPS_FIX_GPS_STATUS = 1 << 0,
    GPS_FIX_POSITION = 1 << 1,
    GPS_FIX_TRACK = 1 << 2,
    GPS_FIX_GROUND_SPEED = 1 << 3,
    GPS_FIX_VERTICAL_SPEED = 1 << 4,
    GPS_FIX_ORIENTATION = 1 << 5,
    GPS_FIX_GPS_TIME = 1 << 6,
    GPS_FIX_DOP = 1 << 7,
    GPS_FIX_POS_COV_TYPE = 1 << 8,
  };
};

//**************************************************

struct CorrImuMeta {
  int64_t start_timestamp_us;
  int64_t finish_timestamp_us;
};

struct Oem7Header {
  char message_name[SENSOR_MESSAGE_NAME_SIZE];
  /// This is the Message ID number of the log (see the
  /// command or log descriptions for the Message ID values
  /// of individual commands or logs)
  // Bits 0-4 = Measurement source1
  /// Bits 5-6 = Format
  /// 00 = Binary
  /// 01 = ASCII
  /// 10 = Abbreviated ASCII, NMEA
  /// 11 = Reserved
  /// Bit 7 = Response bit (see Message Responses on page 45)
  /// 0 = Original Message
  /// 1 = Response Message
  uint16_t message_id;
  uint8_t message_type;
  /// Used for multiple related logs. It is a number
  /// that counts down from N-1 to 0 where N is the
  /// number of related logs and 0 means it is the
  /// last one of the set. Most logs only come out one
  /// at a time in which case this number is 0 .
  uint32_t sequence_number;
  /// The value indicates the quality of the GPS reference time
  uint8_t time_status;
  // GPS reference week number
  uint16_t gps_week_number;
  /// Seconds from the beginning of the GPS reference
  ///  week; accurate to the millisecond level
  uint32_t gps_week_milliseconds;
};
/**
 * @brief The CORRIMUS log contains the RAWIMU data corrected
 *The CORRIMUS log contains the RAWIMU data corrected for gravity, the earth’s
 *rotation and estimated sensor errors. The values in this log are incremental
 *values, accumulated over the logging interval of CORRIMUS, in units of radians
 *for the attitude rate and m/s for the accelerations. Data output is not in the
 *IMU Body frame, but is automatically rotated into the user configured output
 *frame.
 */
struct CorrImu : public ad_std::MessageBase {
  ad_std::HeaderPOD header;
  CorrImuMeta meta;
  uint64_t available;
  Oem7Header oem7_header_corr_imu;
  /// Count of the number of IMU Samples used in each
  /// log output accumulation.
  uint32_t imu_data_count;
  /// right-handed of X-Y-Z(rad/sample)
  ad_std::Vector3d rate;
  /// right-handed of X-Y-Z(m/s/sample)
  ad_std::Vector3d acc;
  ad_std::OrientationWithCovariance orientation_info;
  ad_std::LinearAccelerationWithCovariance linear_acceleration_info;
  ad_std::AngularVelocityWithCovariance angular_velocity_info;

  enum : uint64_t {
    CORR_IMU_OEM7_HEADER_CORR_IMU = 1 << 0,
    CORR_IMU_IMU_DATA_COUNT = 1 << 1,
    CORR_IMU_RATE = 1 << 2,
    CORR_IMU_ACC = 1 << 3,
    CORR_IMU_ORIENTATION_INFO = 1 << 4,
    CORR_IMU_LINEAR_ACCELERATION_INFO = 1 << 5,
    CORR_IMU_ANGULAR_VELOCITY_INFO = 1 << 6,
  };
};
//************************************
struct InsMeta {
  int64_t start_timestamp_us;
  int64_t finish_timestamp_us;
};

/**
 * @brief Inspv
 * This log allows INS position, velocity and attitude, with respect to the SPAN
 * frame, to be collected in one log, instead of using three separate logs
 */

struct InspvaPosition {
  double latitude;  ///< Latitude (WGS84) [degrees]
  double longitude; ///< Longitude (WGS84) [degrees]
  double height;    ///< Ellipsoidal Height (WGS84) [m]
};

struct InsVelocity {
  uint8_t avaliable;
  double north_velocity; ///< North velocity (m/s)
  double east_velocity;  ///< East velocity (m/s)
  double up_velocity;    ///< Up velocity (m/s)

  double north_velocity_stdev; ///< North velocity standard deviation (m/s)
  double east_velocity_stdev;  ///< East velocity standard deviation (m/s)
  double up_velocity_stdev;    ///< Up velocity standard deviation (m/s)

  enum : uint8_t {
    NORTH_VELOCITY = 1 << 0,
    EAST_VELOCITY = 1 << 1,
    UP_VELOCITY = 1 << 2,
    NORTH_VELOCITY_STDEV = 1 << 3,
    EAST_VELOCITY_STDEV = 1 << 4,
    UP_VELOCITY_STDEV = 1 << 5,
  };
};

struct InsOrientation {
  uint8_t avaliable;
  /// Roll in Local Level (degrees)
  double roll;
  /// Pitch in Local Level (degrees)
  double pitch;
  /// Azimuth in Local Level (degrees) This is the inertial azimuth
  /// calculated from the IMU gyros and the SPAN filters.
  double azimuth;
  /// Roll standard deviation (degrees)
  double roll_stdev;
  /// Pitch standard deviation (degrees)
  double pitch_stdev;
  /// Azimuth standard deviation (degrees)
  double azimuth_stdev;

  enum : uint8_t {
    ROLL_VAULE = 1 << 0,
    PITCH_VAULE = 1 << 1,
    AZIMUTH_VALUE = 1 << 2,
    ROLL_STDEV = 1 << 3,
    PITCH_STDEV = 1 << 4,
    AZIMUTH_STDEV = 1 << 5,
  };
};

/// IMU logs are present, but the alignment routine has not started; INS
/// isinactive.
enum class InsStatus : uint8_t {

  INS_INACTIVE = 0,
  /// INS is in alignment mode.
  INS_ALIGNING,
  /// The INS solution is in navigation mode but the azimuth solution
  /// uncertainty has exceeded the threshold. The default threshold is 2
  /// degrees for most IMUs. The solution is still valid but you should
  /// monitor the solution uncertainty in the INSSTDEV log (see page 1019).
  /// You may encounter this state during times when the GNSS, used to aid
  /// the INS, is absent.
  INS_HIGH_VARIANCE,
  /// The INS filter is in navigation mode and the INS solution is good.
  INS_SOLUTION_GOOD,
  /// The INS filter is in navigation mode and the GNSS solution is suspected
  /// to be in error.
  INS_SOLUTION_FREE,
  /// The INS filter is in navigation mode, but not enough vehicle dynamics
  /// have been experienced for the system to be within specifications.
  INS_ALIGNMENT_COMPLETE,
  /// INS is determining the IMU axis aligned with gravity.
  DETERMINING_ORIENTATION,
  /// The INS filter has determined the IMU orientation and is awaiting an
  /// initial position estimate to begin the alignment process.
  WAITING_INITIAL_POS,
  /// The INS filer has orientation, initial biases, initial position and
  /// validroll/pitch estimated. Will not proceed until initial azimuth is
  /// entered.
  WAITING_AZIMUTH,
  /// The INS filter is estimating initial biases during the first 10 seconds
  /// ofstationary data
  INITIALIZING_BIASES,
  /// The INS filter has not completely aligned, but has detected motion.
  MOTION_DETECT,
};

struct Inspva {
  ad_std::HeaderPOD header;
  InsMeta meta;
  uint64_t available;
  Oem7Header oem7_header_inspv;
  InspvaPosition position;
  InsVelocity velocity;       ///< no value for std dev
  InsOrientation orientation; ///< no value for std dev
  InsStatus status;

  enum : uint64_t {
    INSPVA_OEM7_HEADER_INSPVA = 1 << 0,
    INSPVA_POSITION = 1 << 1,
    INSPVA_VELOCITY = 1 << 2,
    INSPVA_ORIENTATION = 1 << 3,
    INSPVA_STATUS = 1 << 4,
  };
};
//************************************
enum class InsPoseType : uint8_t {
  /// No solution
  NONE = 0,
  /// Position has been fixed by the FIX position command or by position
  /// averaging.
  FIXEDPOS = 1,
  /// Position has been fixed by the FIX height or FIX auto command or by
  /// position averaging
  FIXEDHEIGHT = 2,
  /// Velocity computed using instantaneous Doppler
  DOPPLER_VELOCITY = 8,
  /// Solution calculated using only data supplied by the GNSS satellites
  SINGLE = 16,
  /// Solution calculated using pseudorange differential (DGPS, DGNSS)
  /// corrections
  PSRDIFF = 17,
  /// Solution calculated using corrections from an SBAS satellite
  WAAS = 18,
  /// Propagated by a Kalman filter without new observations
  PROPAGATED = 19,
  /// Single-frequency RTK solution with unresolved, float carrier phase
  /// ambiguities
  L1_FLOAT = 32,
  /// Multi-frequency RTK solution with unresolved, float carrier phase
  ///   ambiguities
  NARROW_FLOAT = 34,
  /// Single-frequency RTK solution with carrier phase ambiguities resolved to
  /// integers
  L1_INT = 48,
  /// Multi-frequency RTK solution with carrier phase ambiguities resolved to
  ///   wide - lane integers
  WIDE_INT = 49,
  /// Multi-frequency RTK solution with carrier phase ambiguities resolved to
  /// narrow-lane integers
  NARROW_INT = 50,
  /// RTK status where the RTK filter is directly initialized from the INS
  /// filter
  RTK_DIRECT_INS = 51,
  /// INS position, where the last applied
  ///   position update used a GNSS solution computed using corrections from an
  ///   SBAS(WAAS) solution
  INS_SBAS = 52,
  /// INS position, where the last applied position update used a single point
  /// GNSS (SINGLE) solution
  INS_PSRSP = 53,
  /// INS position, where the last applied position update used a pseudorange
  ///   differential GNSS(PSRDIFF) solution
  INS_PSRDIFF = 54,
  /// INS position, where the last applied
  ///   position update used a floating ambiguity RTK(L1_FLOAT or NARROW_FLOAT)
  ///   solution
  INS_RTKFLOAT = 55,
  /// INS position, where the last applied position update used a fixed integer
  /// ambiguity RTK (L1_INT, WIDE_INT or NARROW_INT) solution
  INS_RTKFIXED = 56,
  /// Converging TerraStar-C, TerraStar-C PRO or TerraStar-X solution
  PPP_CONVERGING = 68,
  /// Converged TerraStar-C, TerraStar-C PRO or TerraStar-X solution
  PPP = 69,
  /// Solution accuracy is within UAL operational limit
  OPERATIONAL = 70,
  /// Solution accuracy is outside UAL operational limit but within warning
  /// limit
  WARNING = 71,
  /// Solution accuracy is outside UAL limits
  OUT_OF_BOUNDS = 72,
  /// INS position, where the last applied position update used a converging
  /// TerraStar-C, TerraStar-C PRO or TerraStar-X PPP (PPP_CONVERGING) solution
  INS_PPP_CONVERGING = 73,
  /// INS position, where the last applied position update used a converged
  ///   TerraStar - C,TerraStar - C PRO or TerraStar - X PPP(PPP) solution
  INS_PPP = 74,
  /// Converging TerraStar-L solution
  PPP_BASIC_CONVERGING = 77,
  /// Converged TerraStar-L solution
  PPP_BASIC = 78,
  /// INS position, where the last applied position update used a converging
  ///   TerraStar - L PPP(PPP_BASIC) solution
  INS_PPP_BASIC_CONVERGING = 79,
  /// INS position, where the last applied position update used a
  /// converged TerraStar - L PPP(PPP_BASIC) solution
  INS_PPP_BASIC = 80,
};

struct InspvaxPosition {
  /// Latitude [degrees]
  double latitude;
  /// Longitude [degrees]
  double longitude;
  /// Height above mean sea level (m)
  double height;
  /// Undulation (m) the relationship between the geoid and
  /// the ellipsoid (m) of the chosen datum
  double undulation;
  /// Latitude standard deviation (meters)
  double latitude_stdev;
  /// Longitude standard deviation (meters)
  double longitude_stdev;
  /// Height standard deviation (meters)
  double height_stdev;
};

/**
 * @brief Inspavx
 * This log includes the information from the INSPVA log, as well as
 * information about the position standard deviation. The position type and
 * solution status fields indicate whether or not the corresponding data is
 * valid.
 */
struct Inspavx : public ad_std::MessageBase {
  ad_std::HeaderPOD header;
  InsMeta meta;
  uint64_t available;
  Oem7Header oem7_header_inspvx;
  InsStatus status;
  InsPoseType pos_type;
  InspvaxPosition position;
  InsVelocity velocity;
  InsOrientation orientation;
  /// OEM7 Commands and Logs Reference Manual v14,p989,Table 226: Extended
  /// Solution Status
  uint32_t ext_sol_status;
  /// Elapsed time since the last ZUPT or position update (seconds)
  uint16_t time_since_update;

  enum : uint64_t {
    INSPV_X_OEM7_HEADER_INSPVX = 1 << 0,
    INSPV_X_INS_STATUS = 1 << 1,
    INSPV_X_POS_TYPE = 1 << 2,
    INSPV_X_POSITION = 1 << 3,
    INSPV_X_VELOCITY = 1 << 4,
    INSPV_X_ORIENATION = 1 << 5,
    INSPV_X_EXT_SOL_STATUS = 1 << 6,
    INSPV_X_TIME_SINCE_UPDATE = 1 << 7,
  };
};
//************************************
struct BestGnssPosMeta {
  int64_t start_timestamp_us;
  int64_t finish_timestamp_us;
};

/**
 * @brief BestGnssPos,Best GNSS Position
 * This log contains the best available GNSS position (without INS) computed by
 * the receiver. In addition, it reports several status indicators, including
 * differential age, which is useful in predicting anomalous behavior brought
 * about by outages in differential corrections. A differential age of 0
 * indicates that no differential correction was used.
 */
enum class BestGnssPosSolStatus : uint8_t {
  /// Solution computed
  SOL_COMPUTED = 0,
  /// Insufficient observations
  INSUFFICIENT_OBS = 1,
  /// No convergence
  NO_CONVERGECE = 2,
  /// Singularity at parameters matrix
  SINGULARITY = 3,
  /// Covariance trace exceeds maximum (trace > 1000 m)
  COV_TRACE = 4,
  /// Test distance exceeded (maximum of 3 rejections if distance >10 km)
  TEST_DIST = 5,
  /// Not yet converged from cold start
  COLD_START = 6,
  /// Height or velocity limits exceeded (in accordance with export licensing
  /// restrictions)
  V_H_LIMIT = 7,
  /// Variance exceeds limits
  VARIANCE = 8,
  /// Residuals are too large
  RESIDUALS = 9,
  /// Large residuals make position unreliable
  INTEGRITY_WARNING = 13,
  /// When a FIX position command is entered, the receiver computes its own
  /// position and determines if the fixed position is valid
  PENDING = 18,
  /// The fixed position, entered using the FIX //position command, is not
  /// valid
  INVALID_FIX = 19,
  /// Position type is unauthorized
  UNAUTHORIZED = 20,
  /// The selected logging rate is not supported for this solution type.
  INVALID_RATE = 22,
};

struct BestGnssPosSatStatus {
  uint8_t num_svs; ///< Number of satellites tracked
  /// Number of satellite solutions used in solution Number of satellites with
  /// L1/E1/B1 signals used in solution
  uint8_t num_sol_svs;
  uint8_t num_sol_l1_svs;
  /// Number of satellites with multi-frequency signals used in solution
  uint8_t num_sol_multi_svs;
};

struct BestGnssPosAge {
  double diff_age; ///< Differential age in seconds
  double sol_age;  ///< Solution age in seconds
};

struct BestGnssPos : public ad_std::MessageBase {
  ad_std::HeaderPOD header;
  BestGnssPosMeta meta;
  uint64_t available;
  Oem7Header oem7_header_best_gnss_pos;
  BestGnssPosSolStatus sol_status;
  InsPoseType pos_type;
  InspvaxPosition position;
  /// Datum ID 61 = WGS84 63 = USER
  double datum_id;
  /// Base station ID
  char stn_id[SENSOR_ID_SIZE];
  BestGnssPosSatStatus sat_status;
  BestGnssPosAge age;
  /// If an RTK solution: an RTK solution has been verified.If a PDP solution:
  /// solution is GLIDE. OEM7 Commands and Logs Reference Manual v14,p457,Table
  /// 84: Extended Solution Status
  uint8_t ext_sol_status;
  /// Galileo and BeiDou signals used mask
  uint8_t galileo_beidou_sig_mask;
  /// GPS and GLONASS signals used mask
  uint8_t gps_glonass_sig_mask;

  enum : uint64_t {
    BEST_GNSS_POS_OEM7_HEADER_BEST_GNSS_POS = 1 << 0,
    BEST_GNSS_POS_SOL_STATUS = 1 << 1,
    BEST_GNSS_POS_POS_TYPE = 1 << 2,
    BEST_GNSS_POS_POSITION = 1 << 3,
    BEST_GNSS_POS_DATUM_ID = 1 << 4,
    BEST_GNSS_POS_STN_ID = 1 << 5,
    BEST_GNSS_POS_SAT_STATUS = 1 << 6,
    BEST_GNSS_POS_AGE = 1 << 7,
    BEST_GNSS_POS_EXT_SOL_STATUS = 1 << 8,
    BEST_GNSS_POS_GALILEO_BEIDOU_SIG_MASK = 1 << 9,
    BEST_GNSS_POS_GPS_GLONASS_SIG_MASK = 1 << 10,
  };
};

} // namespace ad_ins
} // namespace ad_sensor

#endif
