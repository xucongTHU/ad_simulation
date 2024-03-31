
//
// Created by xucong on 23-8-10.
//
#pragma once
#include "cm/cm.h"
#include "pattern/task.hpp"
#include <yaml-cpp/yaml.h>
#include "sim_interface.h"
#include "ad_interface.h"
#include "common/time/timer.h"
#include "proto/common/vehicle_config.pb.h"
#include "common/adapters/adapter_gflags.h"
#include "app/core/core_dependencies/common/vehicle_config_helper.h"

namespace stoic::app::core {
/**
 * @class SimChassis
 * @brief A module subscribe carsim vehicle states with bridge module, and
 * publish vehicle states to stoic chassis and localization.
 */
class CanbusCore : public pattern::Task<CanbusCore> {
 public:
  template<typename... OptionArgs>
  CanbusCore(stoic::cm::NodeHandle& nh, OptionArgs&&... option_args) :
  Task(nh, std::forward<OptionArgs>(option_args)...) {
    nh_ptr_ = &nh;
  }
  CanbusCore(stoic::cm::NodeHandle& nh, const pattern::TaskOptions& task_options) : Task(nh, task_options) {
    nh_ptr_ = &nh;
  }
  ~CanbusCore() = default;
  void ChassisDriverCallback(const std::shared_ptr<sim_chassis::Chassis>& msg);
  void run() override;
 private:
  bool PublishSimChassis(caic_sensor::Canbus* chassis_);
 private:
  stoic::cm::NodeHandle* nh_ptr_;
  sim_chassis::Chassis latest_sim_chassis;
  stoic::cm::proto::common::VehicleParam vehicle_param_;
  int64_t last_sim_chassis_timestamp_ = 0;
  int64_t seq_num = 0;
  bool is_sim_chassis_ready = false;
};
} // namespace stoic::app::core
WORKFLOW_ADD_TASK(::stoic::app::core::CanbusCore)
