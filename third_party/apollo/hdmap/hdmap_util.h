/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/

#pragma once

#include <memory>
#include <string>
#include <iostream>
#include <mutex>
#include "third_party/smartsim/hdmap/hdmap.h"
#include "third_party/smartsim/proto/hdmap/map_id.pb.h"
#include "third_party/smartsim/common/config_gflags.h"
#include "third_party/smartsim/common/file.h"


/**
 * @namespace smartsim::hdmap
 * @brief smartsim::hdmap
 */
namespace smartsim {
namespace hdmap {

/**
 * @brief create a Map ID given a string.
 * @param id a string id
 * @return a Map ID instance
 */
inline smartsim::hdmap::Id MakeMapId(const std::string& id) {
  smartsim::hdmap::Id map_id;
  map_id.set_id(id);
  return map_id;
}

std::vector<std::string> StrSplit(const std::string& files, const std::string& c);

std::string RoutingMapFile();

std::string BaseMapFile();

std::unique_ptr<HDMap> CreateMap(const std::string& map_file_path);

class HDMapUtil {
 public:
  // Get default base map from the file specified by global flags.
  // Return nullptr if failed to load.
  static const HDMap* BaseMapPtr(const std::string&);
  // Guarantee to return a valid base_map, or else raise fatal error.
  static const HDMap& BaseMap(const std::string&);

  //!!must be initialized by above functions BaseMapPtr or BaseMap!!!!
  static const HDMap* BaseMapPtr();

  // Reload maps from the file specified by global flags.
  static bool ReloadMaps(const std::string&);

 private:
  HDMapUtil() = delete;

  static std::unique_ptr<HDMap> base_map_;
  static std::mutex base_map_mutex_;
};

}  // namespace hdmap
}  // namespace smartsim
