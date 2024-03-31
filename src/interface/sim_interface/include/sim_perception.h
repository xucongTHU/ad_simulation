#ifndef SIM_INTERFACE_SIM_PERCEPTION_INTERFACE_H
#define SIM_INTERFACE_SIM_PERCEPTION_INTERFACE_H

#include <stdint.h>

#include <string>
#include <vector>

#include "sim_interface/sim_std.h"

namespace sim_perception {
struct PerceptionMeta {
  int64_t sensor_timestamp_us;
  int64_t start_timestamp_us;
  int64_t finish_timestamp_us;
};

/**
 * @brief  Obstacle info
 *
 */
struct ObstaclePoseInfo {
  sim_std::Point3d position;      /**< position is xyz*/
  sim_std::Vector3d velocity;     /**< the object velocity*/
  sim_std::Vector3d acceleration; /**< the object acceleration*/
  double heading;                  /**< the object heading*/
};

/**
 * @brief Perception Object
 *
 */
struct PerceptionObject {
  /**unique identification of the object*/
  uint32_t id;
  /**the object type*/
  uint8_t type;
  /**the object subtype*/
  uint8_t category;
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

  enum : uint64_t {
    PERCEPTION_OBJECT_ID = 1 << 0,
    PERCEPTION_OBJECT_TYPE = 1 << 1,
    PERCEPTION_OBJECT_CATEGORY = 1 << 2,
    PERCEPTION_OBJECT_POSE_IN_WORLD = 1 << 3,
    PERCEPTION_OBJECT_POSE_IN_VCS = 1 << 4,
    PERCEPTION_OBJECT_LENGTH = 1 << 5,
    PERCEPTION_OBJECT_WIDTH = 1 << 6,
    PERCEPTION_OBJECT_HEIGHT = 1 << 7,
  };
};

/**
 * @brief the perceptions objects
 *
 */
struct PerceptionObjects : public sim_std::MessageBase{
  sim_std::HeaderPOD header;
  PerceptionMeta meta;
  std::vector<PerceptionObject> objects[OBJ_SIZE];
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
  sim_std::Point2d points[PERC_LANE_PTS_SIZE]; /**<the lane vcs points */
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
  sim_std::Point2d left_end_point;      /**<the stop line left end point */
  sim_std::Point2d right_end_point;     /**<the stop line right end point */
  StopLineCubicPolynomial poly;          /**<the stop line vcs poly */
  sim_std::Point2d points[PERC_STOPLINE_PTS_SIZE]; /**<the stop line vcs points */
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
struct PerceptionLanes : public sim_std::MessageBase {
  sim_std::HeaderPOD header;
  PerceptionMeta meta;
  uint8_t size;                               /**<actual used size */
  PerceptionLane lanes[PERC_LANES_SIZE];          /**<the perceptions lanes */
  int lanes_size;
  PerceptionStopLine stop_lines[PERC_STOPLINE_SIZE]; /**<the perceptions stop lines */
  int stop_lines_size;
};
//******************************************************************

struct TrafficLight {
  // id
  std::string id;
  float state;
  // color status: @GREEN:0x100000, @YELLOW:0x1000000, @RED:0x10000000
  uint32_t stateMask;
  float cycleTime;
  float duration;
  uint16_t noPhases;
  uint8_t type;
};

/**
 * @brief PerceptionTrafficLights
 * message of  Perception Traffic Lights
 */
struct PerceptionTrafficLights : public sim_std::MessageBase {
  sim_std::HeaderPOD header;
  PerceptionMeta meta;
  TrafficLight traffic_lights[PERC_TL_SIZE];
};

} // namespace sim_perception

#endif
