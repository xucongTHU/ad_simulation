#pragma once

#include "simulator/vtd_integration/front_vision/lane_line_type/lane_fuision_data_type.h"
#include "simulator/vtd_integration/front_vision/lane_line_type/lane.h"

namespace simulator {
namespace vtd_integration {

struct LaneFusionInput {
  std::shared_ptr<Lane> ego_vision_lane = nullptr;
  std::shared_ptr<Lane> ego_bev_lane = nullptr;
  std::shared_ptr<Lane> ego_hdmap_lane = nullptr;
};

struct KalmanObserveInfo {
  Eigen::Matrix<double, OBSERVE_SIZE, 1> observe_info;
  const LaneBoundary* fusion_line_observe = nullptr;
  bool valid_ =false;
  bool lane_change_observed = false;
};

struct KalmanControlInfo {
  Eigen::Matrix<double, CONTROL_SIZE, 1> control_info;
};

}  // namespace lanefusion
}  // namespace staticfusion
