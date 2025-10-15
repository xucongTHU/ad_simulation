#ifndef ad_INTERFACE_ad_PERCEPTION_INTERFACE_H
#define ad_INTERFACE_ad_PERCEPTION_INTERFACE_H

#include <stdint.h>

#include "ad_interface/ad_std.h"

namespace ad_perception {
struct PerceptionMeta {
  int64_t sensor_timestamp_us;
  int64_t start_timestamp_us;
  int64_t finish_timestamp_us;
};

/**
 * @brief the object type
 *
 */
enum class ObstacleType : uint8_t {
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
enum class VehicleLightStatus : uint8_t {
  UNKNOWN = 0,
  LEFT_TURN_LIGHT_ON,         /**< veh light left turn light on */
  RIGHT_TURN_LIGHT_ON,        /**< veh light right turn light on */
  BRAKE_LIGHT_ON,             /**< veh light brake light on */
  ALL_OFF,                    /**< veh light all off */
  LEFT_TURN_BRAKE_LIGHT_ON,   /**< veh light left turn brake light on */
  RIGHT_TURN_BRAKE_LIGHT_ON,  /**< veh light right turn brake light on */
  BRAKE_DOUBLE_JUMP_LIGHT_ON, /**< veh light brake doule jump light on */
  CLEARANCE_LAMP_ON,          /**< veh light clearance lamp on */
  HEAD_LIGHT_ON,              /**< veh light head light on */
};

/**
 * @brief the object is status, filtered、occluded、predicted
 *
 */
enum class TrackingStatus : uint8_t {
  UNKNOWN = 0,
  INIT,
  FILTER,
  OCCLUDE,
  PREDICT,
  LOST,
};

/**
 * @brief  Obstacle info
 *
 */
struct ObstaclePoseInfo {
  ad_std::Point3d position;      /**< position is xyz*/
  ad_std::Vector3d velocity;     /**< the object velocity*/
  ad_std::Vector3d acceleration; /**< the object acceleration*/
  double heading;                  /**< the object heading*/
  bool position_reliable;          /**< the object pose is avaiable*/
  bool velocity_reliable;          /**< the object velocity is avaiable*/
  bool acceleration_reliable;      /**< the object acceleration is avaiable*/
};

/**
 * @brief   traffic obstacle sub type
 *
 */
enum class TrafficObstacleSubType : uint8_t {
  UNKNOWN = 0,
  CONE,
  WATER_HORSE,
  ROAD_PILE,
  LOCK,
  WARNING_TRIANGLE,
};

/**
 * @brief Perception Object
 *
 */
struct obj_variance {
  float h;
  float l;
  float theat;
  float w;
  float x;
  float y;
  float z;
};

struct Object3Dbox {
  ad_std::Point2d lower_lt; /**<the 3Dbox lower left top point*/
  ad_std::Point2d lower_lb; /**<the 3Dbox lower left  bottom point*/
  ad_std::Point2d lower_rb; /**<the 3Dbox lower right top point*/
  ad_std::Point2d lower_rt; /**<the 3Dbox lower right  bottom point*/

  ad_std::Point2d upper_lt; /**<the 3Dbox upper left top point*/
  ad_std::Point2d upper_lb; /**<the 3Dbox upper left  bottom point*/
  ad_std::Point2d upper_rb; /**<the 3Dbox upper right top point*/
  ad_std::Point2d upper_rt; /**<the 3Dbox upper right  bottom point*/
};

struct PerceptionObject {
  /**binary,1 for available,0 for unavailable */
  uint64_t avaliable;
  /**unique identification of the object*/
  uint32_t id;
  /**the object type*/
  ObstacleType type;
  /**the vehicle object subtype*/
  VehicleSubType veh_sub_type;
  /**the cycle object sub type*/
  CyclistSubType cyclist_sub_type;
  /** the sub type of traffic obstacle*/
  TrafficObstacleSubType traffic_obstacle_sub_type;
  /**the detect score of the object*/
  double score;
  /**the object is valid, 1-valid, 0-invalid*/
  bool valid;
  /**the obstacle postion velocity acceleration in world*/
  ObstaclePoseInfo pose_in_world;
  /**the obstacle postion velocity acceleration in vehicle cordinate system*/
  ObstaclePoseInfo pose_in_vcs;
  /**the object lenght, m*/
  double length;
  /**the object width, m*/
  double width;
  /**the object height, m*/
  double height;
  /**the object exists time unit<us>*/
  uint32_t tracking_time;
  /**the object exists age unit<frame num>*/
  uint32_t tracking_age;
  /**the object tracking status*/
  TrackingStatus tracking_status;
  /**the vehicle light status*/
  VehicleLightStatus veh_light_status;

  /**the covariance*/
  ad_std::CovarianceMatrix<double, 5> covariance;
  /**the variance*/
  obj_variance variance;

  /**the object status*/
  float fea_vec[FEA_VEC_SIZE];
  int fea_vec_size;
  bool is_truncated;
  float trunc_rate;
  bool behind_cam;

  float rear_rect_lt_x; /**<the model output rear rect or full rect*/
  float rear_rect_lt_y;
  float rear_rect_br_x;
  float rear_rect_br_y;
  float full_rect_lt_x;
  float full_rect_lt_y;
  float full_rect_br_x;
  float full_rect_br_y;

  Object3Dbox box_3d; /**<the object 3dbox  */

  enum : uint64_t {
    PERCEPTION_OBJECT_ID = 1 << 0,
    PERCEPTION_OBJECT_TYPE = 1 << 1,
    PERCEPTION_OBJECT_VEH_SUB_TYPE = 1 << 2,
    PERCEPTION_OBJECT_CYCLIST_SUB_TYPE = 1 << 3,
    PERCEPTION_OBJECT_TRAFFIC_OBSTACLE_SUB_TYPE = 1 << 4,
    PERCEPTION_OBJECT_SCORE = 1 << 5,
    PERCEPTION_OBJECT_VALID = 1 << 6,
    PERCEPTION_OBJECT_POSE_IN_WORLD = 1 << 7,
    PERCEPTION_OBJECT_POSE_IN_VCS = 1 << 8,
    PERCEPTION_OBJECT_LENGTH = 1 << 9,
    PERCEPTION_OBJECT_WIDTH = 1 << 10,
    PERCEPTION_OBJECT_HEIGHT = 1 << 11,
    PERCEPTION_OBJECT_TRACKING_TIME = 1 << 12,
    PERCEPTION_OBJECT_TRACKING_AGE = 1 << 13,
    PERCEPTION_OBJECT_TRACKING_STATUS = 1 << 14,
    PERCEPTION_OBJECT_VEH_LIGHT_STATUS = 1 << 15,
    PERCEPTION_OBJECT_COVARIANCE = 1 << 16,
    PERCEPTION_OBJECT_VARIANCE = 1 << 17,
    PERCEPTION_OBJECT_FEA_VEC = 1 << 18,
    PERCEPTION_OBJECT_FEA_VEC_SIZE = 1 << 19,
    PERCEPTION_OBJECT_IS_TRUNCATED = 1 << 20,
    PERCEPTION_OBJECT_TRUNC_RATE = 1 << 21,
    PERCEPTION_OBJECT_BEHIND_CAM = 1 << 22,

    // PERCEPTION_OBJECT_REAR_RECT_LT_X = 1 << 24,
    // PERCEPTION_OBJECT_REAR_RECT_LT_Y = 1 << 25,
    // PERCEPTION_OBJECT_REAR_RECT_BR_X = 1 << 26,
    // PERCEPTION_OBJECT_REAR_RECT_BR_Y = 1 << 27,
    // PERCEPTION_OBJECT_FULL_RECT_LT_X = 1 << 28,
    // PERCEPTION_OBJECT_FULL_RECT_LT_Y = 1 << 29,
    // PERCEPTION_OBJECT_FULL_RECT_BR_X = 1 << 30,
    // PERCEPTION_OBJECT_FULL_RECT_BR_Y = 1 << 31,
    PERCEPTION_OBJECT_BOX_3D = 1 << 23,
  };
};

/**
 * @brief the perceptions objects
 *
 */
struct PerceptionObjects : public ad_std::MessageBase {
  ad_std::HeaderPOD header;
  PerceptionMeta meta;
  PerceptionObject objects[OBJ_SIZE];
  int objects_size;
};
//******************************************************************

/**
 * @brief the line type
 *
 */
enum class LaneColor : uint8_t {
  UNKNOWN = 0,
  WHITE,  /**<lane white */
  YELLOW, /**<lane yello */
};

/**
 * @brief the line type
 *
 */
enum class StopLineType : uint8_t {
  UNKNOWN = 0, /**<stop lane type unknown */
  STOP_LINE,   /**<stop line */
  CROSS_WALK,  /**<cross walk */
};

enum class LaneType : uint8_t {
  UNKNOWN = 0,             /**<lane type unknown */
  SOLID,                   /**<lane solid */
  ROAD_EDGE,               /**<lane road edge */
  DASHED,                  /**<lane dashed */
  LEFT_DASHED_RIGHT_SOLID, /**<lane left dashed right solid */
  LEFT_SOLID_RIGHT_DASHED, /**<lane left solid right dashed */
  DOUBLE_DASHED,           /**<lane double dashed */
  DOUBLE_SOLID,            /**<lane double solid */
  BARRIER,                 /**<lane barrier */
  TYPE_INVALID             /**<lane type invalid */
};

/**
 * @brief the lane index
 *
 */
enum class LaneIndex : uint8_t {
  LANE_NOT_ASSIGNED = 0, /**<lane not assigned */
  LANE_LKA_LEFT,         /**<lane lka left */
  LANE_LKA_RIGHT,        /**<lane lka right */
  LANE_NEXT_LEFT,        /**<lane next left */
  LANE_NEXT_RIGHT,       /**<lane next right */
  LANE_NEXT_NEXT_LEFT,   /**<lane next next left */
  LANE_NEXT_NEXT_RIGHT   /**<lane next next right */
};

/**
 * @brief the lane cubic polynomial
 * y=c0+c1*x+c2*x*x+c3*x*x*x
 */
struct LaneCubicPolynomial {
  /**the lane start point x , the closest point is start_x, include front and
   * rear, 0.1 or -0.1 */
  float start_x;
  /**the lane end point x, the furth point is end_x, include front and rear,
   * 1000 or -1000 */
  float end_x;
  float c0;
  float c1;
  float c2;
  float c3;
};

/**
 * @brief the stop line cubic polynomial
 * x=c0+c1*y+c2*y*y+c3*y*y*y
 */
struct StopLineCubicPolynomial {
  /**the stop line start point y, the far right point is start_y. */
  float start_y;
  /**the stop line end point y, the far left point is end_y. */
  float end_y;
  float c0;
  float c1;
  float c2;
  float c3;
};

/**
 * @brief the perceptions lane
 *
 */
struct PerceptionLane {
  uint64_t avaliable;       /**<binary,1 for available,0 for unavailable */
  uint32_t id;              /**unique identification of the object*/
  LaneColor color;          /**the lane color type*/
  LaneType type;            /**<the lane type */
  LaneIndex index;          /**<the lane index */
  double score;             /**<the lane score */
  LaneCubicPolynomial poly; /**<the lane vcs poly */
  ad_std::Point2d points[PERC_LANE_PTS_SIZE]; /**<the lane vcs points */
  int points_size;
  bool is_predict;                       /**<the lane is predict or not */
  float width;                           /**<lane width, m */
  StopLineType stop_line_type;

  enum : uint64_t {
    PERCEPTION_LANE_ID = 1 << 0,
    PERCEPTION_LANE_COLOR = 1 << 1,
    PERCEPTION_LANE_TYPE = 1 << 2,
    PERCEPTION_LANE_INDEX = 1 << 3,
    PERCEPTION_LANE_SCORE = 1 << 4,
    PERCEPTION_LANE_POLY = 1 << 5,
    PERCEPTION_LANE_POINTS = 1 << 6,
    PERCEPTION_LANE_POINTS_SIZE = 1 << 7,
    PERCEPTION_LANE_IS_PREDICT = 1 << 8,
    PERCEPTION_LANE_WIDTH = 1 << 9,
    PERCEPTION_LANE_STOP_LINE_TYPE = 1 << 10,
  };
};
/**
 * @brief the perceptions stop line
 *
 */
struct PerceptionStopLine {
  uint64_t avaliable; /**<binary,1 for available,0 for unavailable */
  uint32_t id;        /**unique identification of the object*/
  double score;       /**<the stop line score */
  bool is_predict;    /**<the stop line is predict or not */
  float width;        /**<stop line width, m */
  ad_std::Point2d left_end_point;      /**<the stop line left end point */
  ad_std::Point2d right_end_point;     /**<the stop line right end point */
  StopLineCubicPolynomial poly;          /**<the stop line vcs poly */
  ad_std::Point2d points[PERC_STOPLINE_PTS_SIZE]; /**<the stop line vcs points */
  int points_size;
  StopLineType type;

  enum : uint64_t {
    PERCEPTION_STOP_LINE_ID = 1 << 0,
    PERCEPTION_STOP_LINE_SCORE = 1 << 1,
    PERCEPTION_STOP_LINE_IS_PREDICT = 1 << 2,
    PERCEPTION_STOP_LINE_WIDTH = 1 << 3,
    PERCEPTION_STOP_LINE_LEFT_END_POINT = 1 << 4,
    PERCEPTION_STOP_LINE_RIGHT_END_POINT = 1 << 5,
    PERCEPTION_STOP_LINE_POLY = 1 << 6,
    PERCEPTION_STOP_LINE_POINTS = 1 << 7,
    PERCEPTION_STOP_LINE_POINTS_SIZE = 1 << 8,
    PERCEPTION_STOP_LINE_TYPE = 1 << 9,
  };
};
/**
 * @brief  PerceptionLanes
 *message of  the perceptions lane
 */
struct PerceptionLanes : public ad_std::MessageBase {
  ad_std::HeaderPOD header;
  PerceptionMeta meta;
  uint8_t size;                               /**<actual used size */
  PerceptionLane lanes[PERC_LANES_SIZE];          /**<the perceptions lanes */
  int lanes_size;
  PerceptionStopLine stop_lines[PERC_STOPLINE_SIZE]; /**<the perceptions stop lines */
  int stop_lines_size;
};
//******************************************************************

struct TrafficLight {
  /// binary,1 for available,0 for unavailable
  uint64_t avaliable;
  enum struct Color : uint8_t {
    UNKNOWN,
    RED,
    YELLOW,
    GREEN,
    BLACK,
  };
  enum struct ShowType : uint8_t {
    UNKNOWN = 0,
    LEFT_ARROW,
    RIGHT_ARROW,
    FORWARD_ARROW,
    PED,
    VEHICLE,
    TURN_AROUND,
    BICYCLE,
    TIME,
  };
  /// color status: unknown,red,yellow,green
  Color color;
  /// turn towards status:left turn,right turn...
  ShowType show_type;
  /// Traffic light char[]-ID in the map data.
  char id[PERC_TL_ID_SIZE];
  /// How confidence about the detected results, between 0 and 1.
  double confidence;
  /// Is traffic blinking
  bool blink;
  /// v2x traffic light remaining time. unit<us>
  int32_t remaining_time;
  enum : uint64_t {
    TRAFFIC_LIGHT_COLOR = 1 << 0,
    TRAFFIC_LIGHT_SHOW_TYPE = 1 << 1,
    TRAFFIC_LIGHT_ID = 1 << 2,
    TRAFFIC_LIGHT_CONFIDENCE = 1 << 3,
    TRAFFIC_LIGHT_BLINK = 1 << 4,
    TRAFFIC_LIGHT_REMAINING_TIME = 1 << 5,
  };
};

/**
 * @brief PerceptionTrafficLights
 * message of  Perception Traffic Lights
 */
struct PerceptionTrafficLights : public ad_std::MessageBase {
  ad_std::HeaderPOD header;
  PerceptionMeta meta;
  TrafficLight traffic_lights[PERC_TL_SIZE];
  uint8_t size; /**<actual used size */
};

//************************************************************************
// Surround vision perception
struct SurroundPoint {
  ad_std::Point2f ipm_point; /**<point(x,y) about IPM coordinate */
  ad_std::Point2f vcs_point; /**<point(x,y) about vcs coordinate */
};

/**
 * @brief freespace
 */
struct FreespacePoint {
  SurroundPoint pt;
  uint8_t type; /**< freespace point type,0 for background,1 for curb, 2 for
                   obstacle */
};

struct FreespaceResult {
  FreespacePoint free_space_points[FS_PTS_SIZE];
  int free_space_points_size;
};

struct PerceptionFreespacePoint {
  ad_std::Point2f vcs_point; /**<point(x,y) about vcs coordinate */
  uint8_t type; /**< freespace point type,0 for background,1 for curb, 2 for
                   obstacle */
};

struct PerceptionFreespacePoints : public ad_std::MessageBase {
  ad_std::HeaderPOD header;
  PerceptionFreespacePoint freespace_points[PERC_FS_PTS_SIZE];
  int freespace_points_size;
};

/**
 * @brief ipm mask
 */
struct IPMMaskContour {
  SurroundPoint ipm_points[SEG_CT_PTS];
  int ipm_points_size;
};

struct IPMMaskClass {
  IPMMaskContour ipm_contours[SEG_CONTOURS];
  int ipm_contours_size;
};

struct IPMMaskResult {
  IPMMaskClass ipm_classes[SEG_CLASS];
  int ipm_classes_size;
};
/**
 * @brief lanemark
 */
struct LanemarkPoint {
  SurroundPoint pt;
  float score;  /**< confidence score about the keypointpoint detected results,
                   between 0 and 1 */
  uint8_t type; /**< lanemark keypoint type,0 for head keypoint,1 for middle
                   keypoint, 2 for tail keypoint */
};

struct LanemarkBox {
  SurroundPoint pt;
  float box_w;  /**<weight of bounding box*/
  float box_h;  /**<height of bounding box*/
  float score;  /**< confidence score about the lanemark detected results,
                   between 0 and 1 */
  uint8_t type; /**< lanemark type,0 for straight,1 for left, 2 for right, 3 for
                   left-right, 4 for staight-left, 5 for straight-right, 6 for
                   straight-left-right*/
};

struct LanemarkObject {
  int32_t mark_id;                            /**< lanemark id */
  LanemarkPoint lanemark_points[PERC_LANEMARK_OBJ_PTS_SIZE]; /**< lanemark points */
  int lanemark_points_size;
  LanemarkBox lanemark_box;                   /**< lanemark bounding box*/
};
struct LanemarkResult {
  LanemarkObject lanemark_objects[PERC_LANEMARK_OBJ_SIZE];
  int lanemark_objects_size;
};

/**
 * @brief parkingspace
 */
struct SpaceInfo {
  // left_up point of parkingspace in IPM coordinate system
  ad_std::Point2f parkingspace_left_up_point;
  // right_up point of parkingspace in IPM coordinate system
  ad_std::Point2f parkingspace_right_up_point;
  // left_down point of parkingspace in IPM coordinate system
  ad_std::Point2f parkingspace_left_down_point;
  // right_down point of parkingspace in IPM coordinate system
  ad_std::Point2f parkingspace_right_down_point;

  // chock point left, cloest left chock point to left_up point of parkingspace
  ad_std::Point2f chock_left_point;
  // chock point right, cloest right chock point to left_up point of
  // parkingspace
  ad_std::Point2f chock_right_point;
  // angle of left parking-line
  float angle_left;
  // angle of right parking-line
  float angle_right;

  // left_up point of parkingspace sheltered from self car
  bool left_up_point_occ_by_statuscar;
  // left_up point of parkingspace sheltered from obstacle
  bool left_up_point_occ_by_obstacle;
  // which camera does the left-up point appear in ,1 for front camera,2 for
  // back camera, 3 for left camera, 4 for right camera
  uint8_t left_up_point_camera_source;
  // right_up point of parkingspace sheltered from self car
  bool right_up_point_occ_by_statuscar; /**<binary,1 for right-up point
                                           sheltered from owb car,0 for right-up
                                           point not sheltered from owb car */
  // right_up point of parkingspace sheltered from obstacle
  bool right_up_point_occ_by_obstacle;
  // which camera does the right-up point appear in ,1 for front camera,2 for
  // back camera, 3 for left camera, 4 for right camera
  uint8_t right_up_point_camera_source;
};

struct ParkingSpace {
  int64_t timestamp;
  uint32_t track_id;   /**<parking space trackid*/
  uint32_t lost_age;   /**<current parkingspace miss tracked count*/
  SpaceInfo space_vcs; /**<parking space center point of parkingslot in VCS
                          coordinate system*/
  SpaceInfo space_ipm; /**<parking space center point of parkingslot in IPM
                          coordinate system*/
  float score; /**<confidence score about the parkingspace detected results,
                  between 0 and 1 */
  uint8_t
      type; /**<parkingspace type,1 for vertical, 2 for parallel, 3 for slant */
  uint8_t occu;       /**<binary,1 for occupy,0 for not occupy */
  //TODO delete psd_id
};

struct ParkingSpaceResult {
  ParkingSpace parking_spaces[PS_SIZE];
  int parking_spaces_size;
};

/**
 * @brief ipm image trans to vcs(vehichle coordinate, x -> front, y -> left)
 */
struct Ipm2VcsInfo {
  uint16_t x_diff; /**< x_diff in image coordinate */
  uint16_t y_diff; /**< y_diff in image coordinate */
  uint16_t ratio;  /**< ratio */
};

/**
 * @brief PerceptionSurround
 * message of  Perception Surround
 */
struct PerceptionSurround : public ad_std::MessageBase {
  ad_std::HeaderPOD header;
  ParkingSpaceResult parking_space_result; /**<parkingspace result */
  FreespaceResult freespace_result;        /**<freespace result */
  LanemarkResult lanemark_result;          /**<lanemark result */
  ad_std::Image<512, 512, 1> ipm_mask_result; /**<ipm image segementation result */
  PerceptionObjects bev_objects;           /**<bev fisheye detect objects */
  Ipm2VcsInfo info_ipm_vcs;                /**<ipm 2 vcs trans */
};

struct BEVLaneBound {
  enum struct BEVLaneClass : uint8_t {
    UNKNOWN,
    BOUNDRAY,
    DIVIDE,
    CROSS_WALK,
    STOP_LINE,
    OTHER,
    TYPE_INVALID,
  };
  uint64_t avaliable;       /**<binary,1 for available,0 for unavailable */
  BEVLaneClass cls;         /**<the lane class */
  double score;             /**<the lane score */
  LaneCubicPolynomial poly; /**<the lane vcs poly */
  uint32_t index;               /**<reserved*/
  LaneColor color;          /**the lane color type*/
  LaneType type;            /**<the lane type */
  ad_std::Point2d points[PERC_BEV_LB_PTS_SIZE]; /**<the lane vcs points */
  int points_size;
  enum : uint64_t {
    PERCEPTION_LANE_CLS = 1 << 0,
    PERCEPTION_LANE_SCORE = 1 << 1,
    PERCEPTION_LANE_POLY = 1 << 2,
    PERCEPTION_LANE_INDEX = 1 << 3,
    PERCEPTION_LANE_COLOR = 1 << 4,
    PERCEPTION_LANE_TYPE = 1 << 5,
    PERCEPTION_LANE_POINTS = 1 << 6,
    PERCEPTION_LANE_POINTS_SIZE = 1 << 7,
  };
};
struct PerceptionBEVLaneBounds : public ad_std::MessageBase  {
  ad_std::HeaderPOD header;
  PerceptionMeta meta;
  uint8_t size;                    /**<actual used size */
  BEVLaneBound lanes[PERC_BEV_LANES_SIZE]; /**<the bev perceptions lanes */
  int lanes_size;
};
//************************************************************************

} // namespace ad_perception

#endif
