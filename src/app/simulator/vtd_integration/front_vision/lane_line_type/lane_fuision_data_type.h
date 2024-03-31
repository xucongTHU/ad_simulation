#pragma once

#include <memory>

#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Dense"

#include "caic_interface/caic_localization.h"
#include "caic_interface/caic_perception.h"

namespace simulator {
namespace vtd_integration {

const float G_Lane_Fusion_Cycle_Time = 0.1;

static const char* LOG_TAG = "static_fusion";

const unsigned int LINE_STATE_SIZE = 4;
const unsigned int OBSERVE_SIZE = 4;
const unsigned int CONTROL_SIZE = 4;

enum class LaneChangeEvent : uint8_t { NOCHANGE = 0, LEFT_CHANGE = 1, RIGHT_CHANGE = 2};

enum class LaneIndex : uint8_t { EGO_LANE = 0, LEFT_LANE = 1, RIGHT_LANE = 2 };

enum class LaneSuppReason : uint8_t {
  NO_SUPP = 0,
  VEH_SPEEDLOW,
  VEH_SPEEDHIGH,
  LANE_CHANGE,
  LANE_WIDTH,
  LANE_SLOPE,
  LANE_RANGEMIN,
  LANE_RELIABILITY,
  LANE_DATA_LIMITS,
  LANE_EGOLANEMISSING,
  LANE_SINGLINE,
  LANE_VALID,
  LANE_EGOLANESUPP,
  LANE_LOCALIZATION,
  LANE_INCONSTRUCTION,
  LANE_UNPARALLEL
};

enum class Side : uint8_t { Left = 0, Right, SIDE_BOTH,SIDE_UNDEF};

enum class LineSourceType : uint8_t {
  None = 0,
  CAMERA = 1,
  HDMAP = 2,
  CAMERA2HDMAP = 3,
  HDMAP2CAMERA = 4,
  Fusion = 5
};

struct Threshold {
  Threshold() = default;
  Threshold(float lower_input, float upper_input) : lower(lower_input), upper(upper_input) {}
  float lower;
  float upper;
};

struct SuppressionConfig {
  SuppressionConfig()=default;
  SuppressionConfig(const LaneSuppReason& supp_input,const float& deb_start_time,
  const float& deb_end_time){
    supp_reason = supp_input;
    deb_time_start_supp = deb_start_time;
    deb_time_end_supp = deb_end_time ;    
  }
  LaneSuppReason supp_reason ;
  float deb_time_start_supp = 0.0;
  float deb_time_end_supp = 0.0;
  uint8_t supp_priority = 0;
  bool supp_and_relation_ = false;

};
// this is to record the suppresion state
struct SuppressionState {
  bool active_flag = false;
  float deb_time = 0.0;
};

// this is to define the suppression input
struct SuppressionInput{
  //this is for single_side 
  bool start_supp_ = false;
  bool end_supp_ =true; 
  // this is for both side  
  bool left_start_supp = false;
  bool left_end_supp = true;
  bool right_start_supp = false;
  bool right_end_supp = true; 
};


//kalman related info






}  // namespace lanefusion
}
