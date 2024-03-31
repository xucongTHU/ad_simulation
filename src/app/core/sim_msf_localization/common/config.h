//
// Created by xucong on 23-12-22.
//

#ifndef SIM_MSF_LOCALIZATION_COMMON_CONFIG_H_
#define SIM_MSF_LOCALIZATION_COMMON_CONFIG_H_

#include "common/util/yaml_util.h"

namespace stoic::app::core {
class Config {
 public:
  enum class Mode { NORMAL, NOA, AVP, APA, MPA, RAS, UNKNOW };
  static Mode CURRENT_MODE;
};

Config::Mode Config::CURRENT_MODE = Mode::NORMAL;

} //
#endif //SIM_MSF_LOCALIZATION_COMMON_CONFIG_H_
