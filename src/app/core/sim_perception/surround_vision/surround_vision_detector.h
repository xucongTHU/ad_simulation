
//
// Copyright (c) 2024 xucong Authors. All rights reserved.
// Created by xucong on 24-3-13.
//
#ifndef SIM_PERCEPTION_SURROUND_VISION_DETECTOR_H_
#define SIM_PERCEPTION_SURROUND_VISION_DETECTOR_H_
#include "cm/cm.h"
#include "pattern/task.hpp"
#include "sim_interface.h"
#include "ad_interface.h"
#include "common/log/Logger.h"
#include "common/time/timer.h"
#include "pattern/ThreadPool.h"
#include "common/adapters/adapter_gflags.h"
#include "third_party/apollo/proto/geometry/geometry.pb.h"
#include "app/core/sim_perception/surround_vision/postprocess_parkingspace/parkingspace_tracker.h"
#include "app/core/sim_perception/surround_vision/postprocess_freespace/freespace_process.h"

namespace stoic::app::core {
class SurroundVisionDetector : public pattern::Task<SurroundVisionDetector> {
 public:
  template<typename... OptionArgs>
  SurroundVisionDetector(stoic::cm::NodeHandle &nh, OptionArgs &&... option_args) :
      Task(nh, std::forward<OptionArgs>(option_args)...) {
    nh_ptr_ = &nh;
  }
  SurroundVisionDetector(stoic::cm::NodeHandle &nh, const pattern::TaskOptions &task_options) : Task(nh, task_options) {
    nh_ptr_ = &nh;
  }
  ~SurroundVisionDetector() = default;
  void run() override;
 private:
  bool Init();
  void surroundVisionCallback(const std::shared_ptr<sim_perception::PerceptionSurround>& msg);
  void planningStateCallback(const std::shared_ptr<caic_interaction::PlanningState>& msg);
  void poseStateCallback(const std::shared_ptr<caic_localization::LocalizationEstimation>& msg);
  void convert2CaicMsgs(const std::shared_ptr<sim_perception::PerceptionSurround>& output,
                        std::shared_ptr<caic_perception::PerceptionSurround>& msg_caic);
  void convert2CaicMsgs(const std::shared_ptr<sim_perception::PerceptionSurround>& output,
                        std::shared_ptr<caic_world::BigFusion>& msg);
  bool getNearestLane(const apollo::geometry::PointENU& ego_position, double ego_heading,
                      double distance_range, std::string& laneId, apollo::geometry::PointENU& nearestLanePt);
  bool getLocateParkSpace(const apollo::geometry::PointENU& ego_position,std::string& parkspaceId,
                          apollo::geometry::PointENU& leftTopPt, apollo::geometry::PointENU& rightTopPt);
 private:
  stoic::cm::NodeHandle* nh_ptr_;
  std::shared_ptr<caic_localization::LocalizationEstimation> pose_manager = std::make_shared<caic_localization::LocalizationEstimation>();
  std::shared_ptr<sim_perception::PerceptionSurround> perc_surround_;
  std::unique_ptr<ParkingSpaceTracker> parkingspace_tracker_;
  std::unique_ptr<SurroundVisionFreespace> freespace_;
  FreespaceResult freespace_result_;
  bool is_surround_vision_ready = true;
  bool is_pos_ready = false;
  bool is_plan_ready = false;
  std::string sur_pub_str_;
  std::string sur_parkingspace_sv_pub_str_;
  std::string sur_freespace_sv_pub_str_;
  std::string sur_ipm_mask_sv_pub_str_;
  std::string sur_origin_img_sv_pub_str_;
  std::string bev_pub_str_;
  bool hdmap_flag = false;
  std::string hdmap_file;
};
} //
WORKFLOW_ADD_TASK(::stoic::app::core::SurroundVisionDetector)
#endif //SIM_PERCEPTION_SURROUND_VISION_DETECTOR_H_
