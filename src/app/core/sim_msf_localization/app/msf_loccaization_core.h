
//
// Created by xucong on 23-8-10.
//
#pragma once
#include "cm/cm.h"
#include "pattern/task.hpp"
#include "sim_interface.h"
#include "ad_interface.h"
#include <yaml-cpp/yaml.h>
#include "common/time/timer.h"
#include "common/adapters/adapter_gflags.h"
#include "proto/common/string.pb.h"
#include "third_party/apollo/math/euler_angles_zxy.h"
#include "third_party/apollo/proto/geometry/geometry.pb.h"
#include "third_party/apollo/proto/hdmap/map.pb.h"
#include "third_party/apollo/hdmap/hdmap.h"
#include "third_party/apollo/hdmap/hdmap_util.h"
#include "app/core/sim_msf_localization/common/gps_common.h"
#include "app/core/sim_msf_localization/common/config.h"
#include "app/core/sim_msf_localization/common/localization_state.h"

#if defined SD_MAP
#include "maplib/av3hr_interface.h"
#endif

static constexpr double kMapOffsetX = 671147.79;
static constexpr double kMapOffsetY = 3536226.82;
namespace stoic::app::core {
/**
 * @class MsfLocalizationCore
 */
class MsfLocalizationCore : public pattern::Task<MsfLocalizationCore> {
 public:
  template <typename... OptionArgs>
  MsfLocalizationCore(stoic::cm::NodeHandle& nh, OptionArgs&&... option_args) :
  Task(nh, std::forward<OptionArgs>(option_args)...) {
    nh_ptr_ = &nh;
  }
  MsfLocalizationCore(stoic::cm::NodeHandle& nh, const pattern::TaskOptions& task_options) : Task(nh, task_options) {
    nh_ptr_ = &nh;
  }
  ~MsfLocalizationCore() = default;
  void GpsCallback(const std::shared_ptr<sim_localization::Gps>& msg);
  void ImuCallback(const std::shared_ptr<sim_localization::Imu>& msg);
  void MapMsgCallback(const std::shared_ptr<stoic::cm::proto::common::PString>& msg);
  void run() override;
 private:
  void fillLocalizationHeader(caic_localization::LocalizationEstimation* localization);
  bool fillLocalizationAbsPosMsg(const sim_localization::Gps& gps_msg, const sim_localization::Imu& imu_msg,
                                 caic_localization::LocalizationEstimation* localization);
  bool fillLocalizationRelPosMsg(const sim_localization::Gps& gps_msg, const sim_localization::Imu& imu_msg,
                                 caic_localization::LocalizationEstimation* localization);
  bool composeLocalizationMsg(const sim_localization::Gps& gps_msg, const sim_localization::Imu& imu_msg,
                              caic_localization::LocalizationEstimation* localization);
  bool process(caic_localization::LocalizationEstimation& result);
  bool getRelPose(long time);
  double quaternionToHeading(const double qw, const double qx, const double qy, const double qz);
  bool getNearestLane(const apollo::geometry::PointENU& ego_position, double ego_heading,
                      double distance_range, std::string& laneId, apollo::geometry::PointENU& nearestLanePt);
 private:
  stoic::cm::NodeHandle* nh_ptr_;
  sim_localization::Gps gps_msg_{};
  sim_localization::Imu imu_msg_{};
  LocalizationState rel_loc_data_;
  double last_received_timestamp_sec_ = 0.0;
  long current_time = 0;
  int64_t seq_num = 0;
  bool is_gps_ready = false;
  bool is_imu_ready = false;
  bool is_abs_pos_valid = true;
  bool is_rel_pos_valid = true;
  static constexpr long kSecToMicroSec = (1000000l);
#if defined SD_MAP
  caic_map::maplib::Av3hrInterface *map_interface_ = nullptr;
#endif
};
} // namespace stoic::app::core
WORKFLOW_ADD_TASK(::stoic::app::core::MsfLocalizationCore)
