#include "third_party/smartsim/hdmap/hdmap_util.h"

#include <string>
#include <vector>
#include <iostream>


namespace smartsim {
namespace hdmap {

namespace {

// Find the first existing file from a list of candidates: "file_a|file_b|...".
std::string FindFirstExist(const std::string& dir, const std::string& files) {
  const std::vector<std::string> candidates = StrSplit(files, "|");
  for (const auto& filename : candidates) {
    const std::string file_path = FLAGS_map_dir +  "/" + filename;
    if (cyber::common::PathExists(file_path)) {
      return file_path;
    }
  }
  std::cout << "No existing file found in " << dir << "/" << files
         << ". Fallback to first candidate as default result.";
  // ACHECK(!candidates.empty()) << "Please specify at least one map.";
  assert(!candidates.empty());
  return FLAGS_map_dir + "/" + candidates[0];
}

}  // namespace

std::vector<std::string> StrSplit(const std::string& files, const std::string& c) {
  std::string::size_type iter_start, iter_end;
  std::vector<std::string> string_split;
  iter_end = files.find(c);
  iter_start = 0;
  while(iter_end != std::string::npos) {
    string_split.push_back(files.substr(iter_start, iter_end - iter_start));
    iter_start = iter_end + c.size();
    iter_end = files.find(c, iter_start);
  }
  if (iter_start != files.length()) {
    string_split.push_back(files.substr(iter_start));
  }
  return string_split;
}

std::string RoutingMapFile() {
  if (FLAGS_use_navigation_mode) {
    // AWARN << "routing_map file is not used when FLAGS_use_navigation_mode is "
    //          "true";
  }
  return FindFirstExist(FLAGS_map_dir, FLAGS_routing_map_filename);
}

std::string BaseMapFile() {
  if (FLAGS_use_navigation_mode) {
    // AWARN << "base_map file is not used when FLAGS_use_navigation_mode is true";
  }
  return FLAGS_test_base_map_filename.empty()
             ? FindFirstExist(FLAGS_map_dir, FLAGS_base_map_filename)
             : FindFirstExist(FLAGS_map_dir, FLAGS_test_base_map_filename);
}


std::unique_ptr<HDMap> CreateMap(const std::string& map_file_path) {
  std::unique_ptr<HDMap> hdmap(new HDMap());
  if (hdmap->LoadMapFromFile(map_file_path) != 0) {
    std::cout << "Failed to load HDMap " << map_file_path << std::endl;
    return nullptr;
  }
  return hdmap;
}

std::unique_ptr<HDMap> HDMapUtil::base_map_ = nullptr;
std::mutex HDMapUtil::base_map_mutex_;

const HDMap* HDMapUtil::BaseMapPtr(const std::string& base_map_path) {
  if (base_map_ == nullptr) {
    std::lock_guard<std::mutex> lock(base_map_mutex_);
    if (base_map_ == nullptr) {  // Double check.
      base_map_ = CreateMap(base_map_path);
    }
  }
  return base_map_.get();
}

const HDMap& HDMapUtil::BaseMap(const std::string& base_map_path) { return *BaseMapPtr(base_map_path); }


bool HDMapUtil::ReloadMaps(const std::string& base_map_path) {
  
    std::lock_guard<std::mutex> lock(base_map_mutex_);
    base_map_ = CreateMap(base_map_path);
  

  return base_map_ != nullptr;
}

const HDMap* HDMapUtil::BaseMapPtr() {
  if (base_map_ == nullptr) {
    std::lock_guard<std::mutex> lock(base_map_mutex_);
    if (base_map_ == nullptr) {  // Double check.
      base_map_ = CreateMap(BaseMapFile());
    }
  }
  return base_map_.get();
}


}  // namespace hdmap
}  // namespace smartsim
