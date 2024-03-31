
#include "app/core/core_dependencies/common/vehicle_config_helper.h"

#include <algorithm>
#include <iostream>

namespace stoic::app::core {

stoic::cm::proto::common::VehicleConfig VehicleConfigHelper::vehicle_config_;
bool VehicleConfigHelper::is_init_ = false;

VehicleConfigHelper* VehicleConfigHelper::instance_ = nullptr;

void VehicleConfigHelper::Init() { Init(FLAGS_vehicle_param_path); }

void VehicleConfigHelper::Init(const std::string &config_file) {
  stoic::cm::proto::common::VehicleConfig params;
  if (!common::GetProtoFromFile(config_file, &params)) {
    std::cerr << "Unable to parse vehicle config file " << config_file;
    exit(-1);
  }
  Init(params);
}

void VehicleConfigHelper::Init(const stoic::cm::proto::common::VehicleConfig &vehicle_params) {
  vehicle_config_ = vehicle_params;
  is_init_ = true;
}

const stoic::cm::proto::common::VehicleConfig &VehicleConfigHelper::GetConfig() {
  if (!is_init_) {
    Init();
  }
  return vehicle_config_;
}

}  // namespace stoic::app::core
