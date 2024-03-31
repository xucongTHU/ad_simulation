
//
// Created by xucong on 24-1-3.
//
#ifndef SIM_PERCEPTION_SURROUND_COMMON_H_
#define SIM_PERCEPTION_SURROUND_COMMON_H_
#include "yaml-cpp/yaml.h"
#include "common/log/Logger.h"

namespace stoic::app::core {
static const char* LOG_TAG = "surround vision";
const float PARKINGSPACE_HEIGHT = 550.0f;
const float PARKINGSPACE_WIDTH = 240.0f;
template<typename _T>
void loadNode(YAML::Node &node, _T &out, std::string name) {
  if (node[name] != nullptr) {
    out = node[name].as<_T>();
  } else {
    LOG_WARN("Cannot load %s from config", name.c_str());
  };
}
}
#endif //SIM_PERCEPTION_SURROUND_COMMON_H_
