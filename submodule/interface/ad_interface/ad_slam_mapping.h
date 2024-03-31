#ifndef ad_INTERFACE_ad_SLAM_MAPPING_INTERFACE_H
#define ad_INTERFACE_ad_SLAM_MAPPING_INTERFACE_H

#include <algorithm>
#include <string>
#include <vector>

#include "ad_std.h"

namespace ad_slam_mapping {

  #define SLAM_LANEMARKER_PT_SIZE 30
  #define SLAM_CURVE_PT_SIZE 1000
  #define SLAM_CROSSWALK_PT_SIZE 1000
  #define HPA_SLAM_MAP_PATH_SIZE 500
  #define HPA_SLAM_MAP_PSD_SIZE 50
  #define HPA_SLAM_MAP_SPEEDBUMP_SIZE 50
  #define HPA_SLAM_MAP_LANEMARKER_SIZE 50
  #define HPA_SLAM_MAP_DASHLANE_SIZE 50
  #define HPA_SLAM_MAP_LANE_SIZE 50
  #define HPA_SLAM_MAP_CURVE_SIZE 50
  #define HPA_SLAM_MAP_CROSSWALK_SIZE 50
  #define HPA_SLAM_MAP_STOPPINGLINE_SIZE 50

  enum class HpaSlamFaultStatus : uint8_t {
    NO_FAULT = 0,         // mapping no fault
    HAS_FAULT,            // has fault
    OVER_SPEED,           // over speed
    INSUFFICIENT_FEATURE, // insufficient feature in map
    REVERSE_DIST_EXCEED,  // reverse distance exceeds the limit
    RESERVED,             // reserved
  };

  enum class HpaSlamStorageStatus : uint8_t {
    UNAVAILABLE = 0,
    AVAILABLE,
  };

  enum class HpaSlamLengthStatus : uint8_t {
    BEYOUND_LENGTH = 0,
    WITHIN_LENGTH,
  };

  struct HpaSlamStatus : public ad_std::MessageBase{
    HpaSlamFaultStatus fault_status;
    HpaSlamStorageStatus storage_status;
    HpaSlamLengthStatus length_status;
  };

  // psd means parking space detection
  struct SlamPsd{
    int map_index;
    int type;
    ad_std::Point3d a; // clock wise 4 pts
    ad_std::Point3d b;
    ad_std::Point3d c;
    ad_std::Point3d d;
  };

  struct SlamSpeedbump{
    int map_index;
    ad_std::Point3d a; // clock wise 4 pts
    ad_std::Point3d b;
    ad_std::Point3d c;
    ad_std::Point3d d;
  };

  struct PointType{
    ad_std::Point3d point;
    int type;
  };

  struct SlamLaneMarker{
    int map_index;
    int type;
    int pts_size;
    PointType pts[SLAM_LANEMARKER_PT_SIZE];
  };

  struct SlamDashLane{
    int map_index;
    ad_std::Point3d a;
    ad_std::Point3d b;
  };

  struct SlamLane{
    int map_index;
    ad_std::Point3d a; // start point of lane
    ad_std::Point3d b; // end point of stopping line
  };

  struct SlamCurve{
    int map_index;
    int pts_size;
    ad_std::Point3d pts[SLAM_CURVE_PT_SIZE];
  };

  struct SlamCrossWalk{
    int map_index;
    int pts_size;
    ad_std::Point3d pts[SLAM_CROSSWALK_PT_SIZE];
  };

  struct SlamStoppingLine{
    int map_index;
    ad_std::Point3d a; // start point of stopping line
    ad_std::Point3d b; // end point of stopping line
  };

  struct SlamMap: public ad_std::MessageBase{

    int64_t timestamp;
    int mapping_state; // 0: not start map; 1:already start mapping; 2:saving map; 3:finish mapping
    int map_path_size;
    char map_path[HPA_SLAM_MAP_PATH_SIZE];
    float distance; //total distance when mapping, unit is meter
    float mapping_process; // mapping process

    ad_std::PoseWithCovariance cur_pose;

    int psd_size;
    SlamPsd psd[HPA_SLAM_MAP_PSD_SIZE]; // point unit is meter
    int speed_bump_size;
    SlamSpeedbump speed_bump[HPA_SLAM_MAP_SPEEDBUMP_SIZE]; // point unit is meter
    int lane_marker_size;
    SlamLaneMarker lane_marker[HPA_SLAM_MAP_LANEMARKER_SIZE]; // point unit is meter
    int dash_lane_size;
    SlamDashLane dash_lane[HPA_SLAM_MAP_DASHLANE_SIZE]; // point unit is meter
    int lane_size;
    SlamLane lane[HPA_SLAM_MAP_LANE_SIZE]; // point unit is meter
    int curve_size;
    SlamCurve curve[HPA_SLAM_MAP_CURVE_SIZE]; // point unit is meter
    int cross_walk_size;
    SlamCrossWalk cross_walk[HPA_SLAM_MAP_CROSSWALK_SIZE]; // unit is meter
    int stopping_line_size;
    SlamStoppingLine stopping_line[HPA_SLAM_MAP_STOPPINGLINE_SIZE]; // unit is meter
  };
}
#endif
