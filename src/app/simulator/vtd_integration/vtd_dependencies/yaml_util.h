//
// Created by xucong on 23-9-15.
//

#pragma once

#include "yaml-cpp/yaml.h"
#include <string>

namespace simulator::vtd_integration {
class YamlUtil {
 public:
  /**
   * @brief Get a string value from the given yaml[key].
   * @return Whether the field exists and is a valid string.
   */
  static YAML::Node GetValue(const YAML::Node &node, const std::string &key);
};


inline YAML::Node YamlUtil::GetValue(const YAML::Node &node, const std::string &key) {
  if (node.IsMap() && node[key]) {
    return node[key];
  }

  if (node.IsMap()) {
    for (const auto& entry : node) {
      const YAML::Node& value = entry.second;
      YAML::Node result = GetValue(value, key);
      if (!result.IsNull()) {
        return result;
      }
    }
  } else if (node.IsSequence()) {
    for (const auto& element : node) {
      YAML::Node result = GetValue(element, key);
      if (!result.IsNull()) {
        return result;
      }
    }
  }

  return YAML::Node();
}
} // namespace simulator::vtd_integration

