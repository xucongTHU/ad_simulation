#ifndef ad_INTERFACE_ad_PREDICTION_INTERFACE_H
#define ad_INTERFACE_ad_PREDICTION_INTERFACE_H

#include "ad_interface/ad_std.h"

#include <stdint.h>

namespace ad_prediction {

struct PredictionMeta {
  int64_t start_timestamp_us;  /*<Start time stamp of the prediction module*/
  int64_t end_timestamp_us;    /*<End time Stamp of the prediction module*/
  int64_t sensor_timestamp_us; /*<Time stamp of the sensor*/
};

/**
 * @brief  Priority of obstacles
 */
enum struct ObstaclePriority : uint8_t {
  CAUTION = 0, /*< Caution level*/
  NORMAL,      /*<Normal level*/
  IGNORE,      /*< Ignore level*/
};

/**
 * @brief  Intention of obstacles
 */
enum struct Intent : uint8_t {
  UNKNOWN = 0, /*<Unknown*/
  STOP,        /*<Stop*/
  CRUISE,      /*<Cruise*/
  CHANGE_LANE, /*<Change lanes*/
};

enum struct Type : uint8_t {
  UNKNOWN = 0,            /*<Unknown*/
  CRUISE,                 /*<Cruise*/
  CRUISE_URBAN,           /*<Urban road cruise*/
  CRUISE_HIGHWAY,         /*<High speed road cruise*/
  JUNCTION,               /*<Junction*/
  JUNCTION_TRAFFIC_LIGHT, /*<Junction traffic light*/
  JUNCTION_STOP_SIGN,     /*<Junction stop signal*/
};

struct Trajectory {
  uint8_t size;                                                   /*<trajectory point size*/
  float probability;                                              /*<Confidence of the trajectory*/
  ad_std::TrajectoryPoint trajectory_points[PRED_TRA_PTS_SIZE]; /*<Collection of track points*/
};

/**
 * @brief Object of prediction
 */
struct PredictionObject {
  bool is_static;                 /*<Static sign of the predicted object*/
  ObstaclePriority priority;      /*<Priority of the predicted object*/
  Intent intent;                  /*<Intention of the predicted object*/
  uint32_t id;                    /*<Object identification*/
  int trajectories_in_world_size; /*<Trajectory of the predicted object's num in worLd coordinates*/
  int trajectories_in_vcs_size;   /*<Trajectory of the predicted object's num in vcs coordinates*/
  float predicted_period;         /*<Predicted duration of the predicted object unit<s>*/
  uint64_t available;             /*<The state flag of the predicted object*/
  uint64_t timestamp;             /*<Timestamp of the predicted object*/

  enum : uint64_t {
    PREDICTION_OBJECT_ID = 1 << 0,                    /*<Predicted object identification*/
    PREDICTION_OBJECT_IS_STATIC = 1 << 1,             /*<Static flag of the predicted object*/
    PREDICTION_OBJECT_TIMESTAMP = 1 << 2,             /*<Timestamp of the predicted object*/
    PREDICTION_OBJECT_PREDICTED_PERIOD = 1 << 3,      /*<Predicted duration of the predicted object*/
    PREDICTION_OBJECT_PRIORITY = 1 << 4,              /*<Priority of the predicted object in worLd coordinates*/
    PREDICTION_OBJECT_INTENT = 1 << 5,                /*<Intention of the predicted object in vcs coordinates*/
    PREDICTION_OBJECT_TRAJECTORIES_IN_WORLD = 1 << 6, /*<Trajectory of the predicted object in*/
    PREDICTION_OBJECT_TRAJECTORIES_IN_VCS = 1 << 7,   /*<Trajectory of the predicted object*/
    PREDICTION_OBJECT_RESERVED_INFO = 1 << 8,         /*<Retention information of the predicted object*/
  };

  char reserved_info[STD_RESERVED_INFO_SIZE];        /*<Retention information of the predicted object*/
  Trajectory trajectories_in_world[PRED_TRA_W_SIZE]; /*<Trajectory of the predicted object in worLd coordinates*/
  Trajectory trajectories_in_vcs[PRED_TRA_V_SIZE];   /*<Trajectory of the predicted object in vcs coordinates*/
};

/**
 * @brief Set of prediction objects
 */
struct PredictionObjects : public ad_std::MessageBase  {
  int prediction_objects_size;
  int reserved_info_size;
  uint64_t available;

  enum : uint64_t {
    PREDICTION_HEADER = 1 << 0,
    PREDICTION_META = 1 << 1,
    PREDICTION_OBJECTS = 1 << 2,
    PREDICTION_OBJECTS_SIZE = 1 << 3,
    PREDICTION_RESERVED_INFO = 1 << 4,
    PREDICTION_RESERVED_INFO_SIZE = 1 << 5
  };

  PredictionMeta meta;
  char reserved_info[STD_RESERVED_INFO_SIZE];          /*<Retention information of the predicted objects*/
  ad_std::HeaderPOD header;                          /*<The message header of the prediction object collection*/
  PredictionObject prediction_objects[PRED_OBJS_SIZE]; /*<Set of prediction objects*/
};
}  // namespace ad_prediction

#endif
