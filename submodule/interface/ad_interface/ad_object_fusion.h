#ifndef ad_INTERFACE_ad_OBJECT_FUSION_INTERFACE_H
#define ad_INTERFACE_ad_OBJECT_FUSION_INTERFACE_H

#include <stdint.h>

#include <string>
#include <vector>

#include "ad_interface/ad_std.h"

#define FUSION_INTERFACE_DEMENSION_3 3

namespace ad_object_fusion {
struct MetaInfo {
  int64_t objects_timestamp_us;
  int number_of_objects;
};

/**
 * @brief the object type
 *
 */
enum class ObjectType : uint8_t {
  UNKNOWN = 0,     /**< unknown */
  VEHICLE,         /**< vehicle */
  PEDESTRIAN,      /**< pedestrian */
  CYCLIST,         /**< cyclist */
  TRAFFIC_LIGHT,   /**< traffic light */
  TRAFFIC_SIGN,    /**< traffic sign */
  TRAFFIC_BARRIER, /**< traffic barrier -cone etc.*/
};

/**
 * @brief the vehicle subtype
 *
 */
enum class VehicleSubType : uint8_t {
  UNKNOWN = 0, /**< unknown */
  CAR,         /**< car */
  SUV,         /**< suv */
  VAN,         /**< van */
  MINI_BUS,    /**< mini bus */
  BUS,         /**< bus */
  TRUCK,       /**< truck */
  BIG_TRUCK,   /**< big track */
  SPECIAL,     /**< special car */
  BIG_VEHICLE, /**< big vehicle,only for lidar perception */
};

/**
 * @brief the cycle subtype
 *
 */
enum class CyclistSubType : uint8_t {
  UNKNOWN = 0, /**< unknown */
  BICYCLE,     /**< bicycle */
  MOTORCYCLE,  /**< motorcycle */
  TRICYCLE,    /**< tricycle */
};

/**
 * @brief vehicle light status
 *
 */
struct VehicleLightStatus {
  bool left_turn_light_on = false;  /**< vehicle turn on the left turn light */
  bool right_turn_light_on = false; /**< vehicle turn on the right turn light */
  bool brake_lights_on = false;     /**< vehicle turn on the brake lights */
  bool warning_lights_on = false;   /**< vehicle turn on the warning lights */
  bool reverse_lights_on = false;   /**< vehicle turn on the reverse lights */
  bool head_lights_on = false;      /**< vehicle turn on the head lights */
  bool fog_lights_on = false;       /**< vehicle turn on the fog lights */
};

/**
 * @brief motion status
 *
 */
enum class MotionStatus : uint8_t {
  UNKNOWN = 0, /**< unknown */
  INVALID,     /**< invalid type */
  MOVING,      /**< moving */
  STATIONARY,  /**< stationary */
  STOPPED,     /**< stopped */
};

/**
 * @brief  Obstacle Pose info
 *
 */
struct ObjectPoseInfo {
  ad_std::Point3d position;      /**< position is xyz*/
  ad_std::Vector3d velocity;     /**< the object velocity*/
  ad_std::Vector3d acceleration; /**< the object acceleration*/

  ad_std::CovarianceMatrix<double, FUSION_INTERFACE_DEMENSION_3>
      covariance_position;     /**< position covariance*/
  ad_std::CovarianceMatrix<double, FUSION_INTERFACE_DEMENSION_3>
      covariance_velocity;     /**< velocity covariance*/
  ad_std::CovarianceMatrix<double, FUSION_INTERFACE_DEMENSION_3>
      covariance_acceleration; /**< acceleration covariance*/

  double heading;              /**< the object heading*/
  double covariance_heading;
};

/**
 * @brief   traffic obstacle sub type
 *
 */
enum class TrafficObstacleSubType : uint8_t {
  UNKNOWN = 0,      /**< unknown */
  CONE,             /**< cone */
  WATER_HORSE,      /**< water horse */
  ROAD_PILE,        /**< road pile */
  LOCK,             /**< lock */
  WARNING_TRIANGLE, /**< warning triangle */
};
/**
 * @brief   fusion object
 *
 */
struct FusionObject {
  /**binary,1 for available,0 for unavailable */
  uint64_t avaliable;
  /**unique identification of the object*/
  uint32_t id;
  /**the object type*/
  ObjectType type;
  /**the vehicle object subtype*/
  VehicleSubType veh_sub_type;
  /**the cycle object sub type*/
  CyclistSubType cyclist_sub_type;
  /** the sub type of traffic obstacle*/
  TrafficObstacleSubType traffic_obstacle_sub_type;

  /**the obstacle postion velocity acceleration in world*/
  ObjectPoseInfo pose_in_world;
  /**the obstacle postion velocity acceleration in vehicle cordinate system*/
  ObjectPoseInfo pose_in_vcs;

  /**the object lenght, m*/
  double length;
  /**the object width, m*/
  double width;
  /**the object height, m*/
  double height;
  /**the object size covariance*/
  ad_std::CovarianceMatrix<double, FUSION_INTERFACE_DEMENSION_3>
      covariance_size;
  /**if the convex is available*/
  bool convex_available = false;
  /**the object convex in world coordinate*/
  ad_std::Polygon2d convex_world;
  /**the object convex in vcs coordinate*/
  ad_std::Polygon2d convex_vcs;

  MotionStatus motion_status;
  /**the object exists time unit<us>*/
  uint32_t tracking_age;
  /**the object tracking quality*/
  double tracking_quality;
  /**the vehicle lights status*/
  VehicleLightStatus veh_light_status;

  enum : uint64_t {
    FUSION_OBJECT_ID = 1 << 0,
    FUSION_OBJECT_TYPE = 1 << 1,
    FUSION_OBJECT_VEH_SUB_TYPE = 1 << 2,
    FUSION_OBJECT_CYCLIST_SUB_TYPE = 1 << 3,
    FUSION_OBJECT_TRAFFIC_OBSTACLE_SUB_TYPE = 1 << 4,
    FUSION_OBJECT_POSE_IN_WORLD = 1 << 5,
    FUSION_OBJECT_POSE_IN_VCS = 1 << 6,
    FUSION_OBJECT_LENGTH = 1 << 7,
    FUSION_OBJECT_WIDTH = 1 << 8,
    FUSION_OBJECT_HEIGHT = 1 << 9,
    FUSION_OBJECT_COVARIANCE_SIZE = 1 << 10,
    FUSION_OBJECT_CONVEX_AVAILABLE = 1 << 11,
    FUSION_OBJECT_CONVEX_WORLD = 1 << 12,
    FUSION_OBJECT_CONVEX_VCS = 1 << 13,
    FUSION_OBJECT_MOTION_STATUS = 1 << 14,
    FUSION_OBJECT_TRACKING_AGE = 1 << 15,
    FUSION_OBJECT_TRACKING_QUALITY = 1 << 16,
    FUSION_OBJECT_VEH_LIGHT_STATUS = 1 << 17,
  };
};

/**
 * @brief the perceptions objects
 *
 */
struct FusionObjects {
  ad_std::Header header;
  MetaInfo meta_info;
  std::vector<FusionObject> objects;
};

} // namespace ad_object_fusion

#endif
