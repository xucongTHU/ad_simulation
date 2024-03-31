//
// Created by xucong on 23-9-15.
//

#ifndef stoic_SRC_COMMON_UTIL_YAML_UTIL_H_
#define stoic_SRC_COMMON_UTIL_YAML_UTIL_H_

#include "yaml-cpp/yaml.h"
#include <string>

namespace stoic::common {
class YamlUtil {
 public:
  /**
   * @brief Get a string value from the given yaml[key].
   * @return Whether the field exists and is a valid string.
   */
  static YAML::Node GetValue(const YAML::Node &yaml, const std::string &key);
};

} // namespace stoic::common

#endif //stoic_SRC_COMMON_UTIL_YAML_UTIL_H_
