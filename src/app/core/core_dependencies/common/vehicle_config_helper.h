#pragma once

#include <string>

#include "proto/common/vehicle_config.pb.h"
#include "proto/geometry/pnc_point.pb.h"
#include "common/util/proto_util.h"


namespace stoic::app::core {

/**
 * @class VehicleConfigHelper
 *
 * @Brief This is a helper class that can load vehicle configurations. The
 * vehicle configurations are
 * defined modules/common/configs/proto/vehicle_config.proto
 */
class VehicleConfigHelper {
 public:
  /**
   * @brief Initialize vehicle configurations with default configuration file
   * pointed by gflags FLAGS_vehicle_config_path. The code will crash if
   * FLAGS_vehicle_config_path does not exist or it points to a file with
   * invalid format.
   */
  static void Init();


  static VehicleConfigHelper* Instance() {
    if (instance_ == nullptr) {
      instance_ = new VehicleConfigHelper;
    }

    return instance_;  
  }                         

  /**
   * @brief Initialize vehicle configurations with \p config
   * @param config A VehicleConfig class instance. The VehicleConfig class is
   * defined by modules/common/configs/proto/vehicle_config.proto.
   */
  static void Init(const stoic::cm::proto::common::VehicleConfig &config);

  /**
   * @brief Initialize vehicle configurations with \p config_file.
   * The code will crash if \p config_file does not exist or \p config_file has
   * invalid format.
   * @param config_file The configuration file path. The format of the file is
   * defined by protobuf file
   * modules/common/configs/proto/vehicle_config.proto.
   */
  static void Init(const std::string &config_file);

  /**
   * @brief Get the current vehicle configuration.
   * @return the current VehicleConfig instance reference.
   */
  static const stoic::cm::proto::common::VehicleConfig &GetConfig();

 private:
  static stoic::cm::proto::common::VehicleConfig vehicle_config_;
  static bool is_init_;
  static VehicleConfigHelper* instance_;
};


}  // namespace stoic::app::core
