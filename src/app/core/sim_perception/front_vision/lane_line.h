
//
// Created by xucong on 24-1-3.
//
#ifndef SIM_PERCEPTION_LANE_LINE_H_
#define SIM_PERCEPTION_LANE_LINE_H_
#include "cm/cm.h"
#include "pattern/task.hpp"
#include "sim_interface.h"
#include "ad_interface.h"
#include "common/log/Logger.h"
#include "common/time/timer.h"
#include "common/adapters/adapter_gflags.h"

namespace stoic::app::core {
 class LaneLine : public pattern::Task<LaneLine> {
 public:
  template <typename... OptionArgs>
  LaneLine(stoic::cm::NodeHandle& nh, OptionArgs&&... option_args) :
    Task(nh, std::forward<OptionArgs>(option_args)...) {
      nh_ptr_ = &nh;
    }
    LaneLine(stoic::cm::NodeHandle& nh, const pattern::TaskOptions& task_options) : Task(nh, task_options) {
      nh_ptr_ = &nh;
    }
  ~LaneLine() = default;
  void PercLanesCallback(const std::shared_ptr<sim_perception::PerceptionLanes>& msg);
  void run() override;
 private:
  void ConvertLaneRes2CAICMsg(const sim_perception::PerceptionLanes& output,
                              std::shared_ptr<caic_perception::PerceptionLanes>& perc_lanes);
 private:
  stoic::cm::NodeHandle* nh_ptr_;
  sim_perception::PerceptionLanes sim_perc_lanes_;
  bool is_sim_perc_ready = false;
  std::unordered_map<sim_perception::LaneType, int> type_convert_ = {
      {sim_perception::LaneType::UNKNOWN, 0 },
      {sim_perception::LaneType::SOLID, 1 },
      {sim_perception::LaneType::ROAD_EDGE, 2},
      {sim_perception::LaneType::DASHED,  3},
      {sim_perception::LaneType::DOUBLE_DASHED,  6},
      {sim_perception::LaneType::DOUBLE_SOLID, 7}};
  std::unordered_map<sim_perception::LaneIndex, std::string> index_convert_ = {
      {sim_perception::LaneIndex::LANE_LKA_LEFT, "L"},
      {sim_perception::LaneIndex::LANE_LKA_RIGHT, "R"},
      {sim_perception::LaneIndex::LANE_NEXT_LEFT, "LL"},
      {sim_perception::LaneIndex::LANE_NEXT_RIGHT, "RR"}};
};
} //namespace stoic::app::core
WORKFLOW_ADD_TASK(::stoic::app::core::LaneLine)
#endif //SIM_PERCEPTION_LANE_LINE_H_
