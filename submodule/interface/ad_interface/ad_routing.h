#ifndef ad_INTERFACE_ad_ROUTING_INTERFACE_H
#define ad_INTERFACE_ad_ROUTING_INTERFACE_H

#include <stdint.h>

#include <vector>

#include "ad_interface/ad_std.h"

namespace ad_routing {

struct RoutingRequestMeta {
  /// start timestamp while receiving pose info.
  int64_t start_timestamp_us;
  /// finish timestamp while generating routing request msg.
  int64_t finish_timestamp_us;
};

enum struct PointType : uint8_t {
  PASSBY = 0,
  DESTINATION = 1,
  PULLOVER = 2,
  STATION = 3,
};

/*!
 * \brief  Lane Waypoint
 * global Lane request waypoint
 */
struct LaneWaypoint {
  /// lane id
  char id[ROUT_ID_SIZE];
  /// s value along frenet-s-axis
  double s;
  /// pose info in ENU coordinate (can find in third
  /// party/apollo/proto/geometry folder)
  ad_std::Point3d pose;
  /// station or ego heading info
  double heading;
  // waypoint type
  PointType type;
  /// waypoint name (for bus/taxi station)
  char name[ROUT_LW_PTS_NAME_SIZE];
};
/*!
 * \brief  Lane Segment
 * List of blacklisted lanes (unavailable lanes)
 */
struct LaneSegment { 
  /// lane id
  char id[ROUT_ID_SIZE];
  /// start s value in frenet-s-axis of this lane segment
  double start_s;
  /// end s value in frenet-s-axis of this lane segment
  double end_s;
};
/*!
 * \brief  ParkingInfo
 * parking space information
 */
struct ParkingInfo {
  /// id of parking space
  char parking_space_id[ROUT_ID_SIZE];
  /// parking point that adc back-axis point should approach.
  ad_std::Point3d parking_point;
};
/**
 * @brief Parking Space
 * ParkingSpace is a place designated to park a car.
 */
struct ParkingSpace {
  char id[ROUT_ID_SIZE];
ad_std::Vector2d parking_space[ROUT_PSPACE_PTS_SIZE];
  int parking_space_size;
  char overlap_id[ROUT_ID_SIZE];
  double heading;
};
enum struct RequestType : uint8_t {
  ROB = 0,
  DEMO = 1,
  CLICK = 2,
  PARK = 3,
};

struct RoutingRequest : public ad_std::MessageBase {
  /// heading info of this routing request
  ad_std::HeaderPOD header;
  /// meta data including start/end timestamp
  RoutingRequestMeta meta;
  /// whether each field value is available or not in binary sequence(1 for
  /// available)
  uint64_t available;
  /// waypoints that adc should drive through.
  LaneWaypoint waypoints[ROUT_WAY_PTS_SIZE];
  int waypoints_size;
  /// lane ids that cannot be used by routing
  LaneSegment blacklisted_lanes[ROUT_BL_LANES_SIZE];
  int blacklisted_lanes_size;
  /// road ids that cannot be used by routing
  char blacklisted_roads [ROUT_BL_ROADS_SIZE][ROUT_BL_ROADS_IFNO_SIZE];
  int blacklisted_roads_size;
  /// parking space for AVP planner
  ParkingSpace parking_space;
  /// parking info for AVP planner
  ParkingInfo parking_info;
  /// request type for different scenario
  RequestType request_type;

  enum : uint64_t {
    ROUTING_REQUEST_WAYPOINTS = 1 << 0,
    ROUTING_REQUEST_WAYPOINTS_SIZE = 1 << 1,
    ROUTING_REQUEST_BLACKLISTED_LANES = 1 << 2,
    ROUTING_REQUEST_BLACKLISTED_LANES_SIZE = 1 << 3,
    ROUTING_REQUEST_BLACKLISTED_ROADS = 1 << 4,
    ROUTING_REQUEST_BLACKLISTED_ROADS_SIZE = 1 << 5,
    ROUTING_REQUEST_PARKING_SPACE = 1 << 6,
    ROUTING_REQUEST_PARKING_INFO = 1 << 7,
  };
};
//************************************************************************

struct RoutingResponseMeta {
  int64_t start_timestamp_us;  ///< timestamp when start to do routing.
  int64_t finish_timestamp_us; ///< finished timestamp for rouring process.
};

enum struct ChangeLaneType : uint8_t {
  FORWARD = 0, ///< no change lane
  LEFT = 1,    ///< change to left lane
  RIGHT = 2,   ///< change to right lane
};

struct Passage {
  LaneSegment segment[ROUT_SEGMENT_SIZE]; ///< lane segment sequence along passage
  int segment_size;
  bool can_exit;                    ///< true for not change lane
  ChangeLaneType change_lane_type;  ///< change lane type (forward/left/right)
};
struct RoadSegment {
  char id[ROUT_ID_SIZE];               ///< road id
  Passage passage[ROUT_PASSAGE_SIZE]; ///< passage sequence that the road contained.
  int passage_size;
};
struct Measurement {
  /// accumulated distance of the whole routing path unit<m>
  double distance;
  /// estimation of time that adc can reach the dest unit<s>.
  double estimated_time;
};

struct RoutingResponse : public ad_std::MessageBase {
  /// header info
  ad_std::HeaderPOD header;
  /// meta data that contains start and end timestamp of routing process.
  RoutingResponseMeta meta;
  /// whether each field value is available or not in  binary sequence(1 for
  /// available)
  uint64_t available;
  /// road sequence that adc need to drive through along routing response.
  RoadSegment road[ROUT_ROAD_SIZE];
  int road_size;
  /// measurement info of the routing path.
  Measurement measurement;
  /// request that contains a list of waypoints for adc driving glong.
  RoutingRequest routing_request;
  /// routing process status,1 for success, -1 for failure
  int32_t status;

  enum : uint64_t {
    ROUTING_RESPONSE_ROAD = 1 << 0,
    ROUTING_RESPONSE_ROAD_SIZE = 1 << 1,
    ROUTING_RESPONSE_MEASUREMENT = 1 << 2,
    ROUTING_RESPONSE_ROUTING_REQUEST = 1 << 3,
    ROUTING_RESPONSE_STATUS = 1 << 4,
  };
};

} // namespace ad_routing

#endif
