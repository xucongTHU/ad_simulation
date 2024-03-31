#ifndef ad_INTERFACE_ad_PLANNING_INTERFACE_H
#define ad_INTERFACE_ad_PLANNING_INTERFACE_H

#include <stdint.h>

#include "ad_interface/ad_std.h"

namespace ad_planning {

struct PlanningMeta {
  int64_t start_timestamp_us; ///<time stamp of starting to plan
  int64_t end_timestamp_us;   ///<time stamp of finishing planning process
};

struct Emergency {
  bool has_estop;            ///<true for emergency stop situation
  uint8_t source_type;       ///<AEB, BSD....
  char reason[PLANNING_EG_REASON_SIZE];        ///<reason for emergency stop.
};
struct Warning {
  uint8_t source_type;       ///<AEB, BSD....
  char content[PLANNING_CONTENT_SIZE];       ///<warning message info
};
/*!
 * \brief  Trajectory type of ego vehicle
 */
enum struct TrajectoryType : uint8_t {
  UNKNOWN = 0,        ///<abnormal type
  NORMAL = 1,         ///<regular trajectory
  PATH_FALLBACK = 2,  ///<path-failed trajectory
  SPEED_FALLBACK = 3, ///<speed-failed trajectory
  PATH_REUSED = 4,    ///<reusing previous path.
};

/*!
 * \brief  Planning decision of current scenario
 */
enum struct Decision : uint8_t {
  CRUISE = 0,
  STOP = 1,
  ESTOP = 2,
  CHANGE_LANE = 3,
  MISSION_COMPLETE = 4,
  NOT_READY = 5,
  PARKING = 6,
  PARK_CRUISE = 7,
};

struct StationStatus {
  bool is_come_to_stop = false;
  bool is_stop = false;
  bool is_stop_go = false;
  uint32_t current_stop_num = 0;
  uint32_t next_stop_num = 0;
};

enum struct TrajectoryStatus : uint8_t {
  COMPLETE = 0,   ///<control both lon and lat
  LONGITUDE = 1,  ///<only lon control
  LATERAL = 2,    ///<only lat control
};

struct PlanningResult : public ad_std::MessageBase {
  ///header info
  ad_std::HeaderPOD header;
  ///meta data including start/end timestamp of planning process
  PlanningMeta meta;
  ///binary value for representing availability of each field.
  uint64_t available;
  ///total length in meter of planning trajctory unit<m>
  double total_path_length;
  ///total time in s of planning trajectory unit<s>
  double total_path_time;
  ///@brief actual used size
  uint8_t trajectory_points_size;
  ///a sequence of points along trajectory, including
  ///x,y,phi,kappa,v,a info
  ad_std::TrajectoryPoint trajectory_points[PLANNING_TRA_PTS_SIZE];
  ///emergency message
  Emergency emergency;
  ///warning message
  Warning warnings[PLANNING_WARNING_SIZE];
  int warnings_size;
  ///whether replan from the moment of this trajectory
  bool is_replan;
  ///replan reason
  char replan_reason[PLANNING_REPLAN_REASON_SIZE];
  ///gear info (e.g. forward/reverse)
  ad_std::GearState gear_state;
  ///type of this trajectory
  TrajectoryType trajectory_type;
  ///vehicle turning info.
  ad_std::VehicleSignal vehicle_signal;
  ///open door command, 0 for no action, 1 for open  
  bool open_door_command;
  ///current planning decision
  Decision decision;
  ///station status for bus
  StationStatus station_status;
  ///reserved field used in future
  char reserved_info[STD_RESERVED_INFO_SIZE];
  ///trajectory lon or lat control status
  TrajectoryStatus trajectory_status;

  enum : uint64_t {
    PLANNING_RESULT_TOTAL_PATH_LENGTH = 1 << 0,
    PLANNING_RESULT_TOTAL_PATH_TIME = 1 << 1,
    PLANNING_RESULT_TRAJECTORY_POINTS_SIZE = 1 << 2,
    PLANNING_RESULT_TRAJECTORY_POINTS = 1 << 3,
    PLANNING_RESULT_EMERGENCY = 1 << 4,
    PLANNING_RESULT_WARNINGS = 1 << 5,
    PLANNING_RESULT_WARNINGS_SIZE = 1 << 6,
    PLANNING_RESULT_IS_REPLAN = 1 << 7,
    PLANNING_RESULT_REPLAN_REASON = 1 << 8,
    PLANNING_RESULT_GEAR_STATE = 1 << 9,
    PLANNING_RESULT_TRAJECTORY_TYPE = 1 << 10,
    PLANNING_RESULT_VEHICLE_SIGNAL = 1 << 11,
    PLANNING_RESULT_OPEN_DOOR_COMMAND = 1 << 12,
    PLANNING_RESULT_DECISION = 1 << 13,
    PLANNING_RESULT_STATION_STATUS = 1 << 14,
  };
};

} // namespace ad_planning

#endif