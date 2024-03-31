
//
// Copyright (c) 2024 xucong Authors. All rights reserved.
// Created by xucong on 24-3-14.
//
#ifndef SURROUND_VISION_POSTPROCESS_FREESPACE_FREESPACE_PROCESS_H_
#define SURROUND_VISION_POSTPROCESS_FREESPACE_FREESPACE_PROCESS_H_
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include "ad_interface.h"
#include "sim_interface.h"
#include "app/core/sim_perception/surround_vision/postprocess_freespace/freespace_common.h"
namespace stoic::app::core {
class SurroundVisionFreespace {
 public:
  /**
   * @brief Constructor
   */
  SurroundVisionFreespace(const std::string &cfg) { Init(cfg); };
  /**
   * @brief Destructor
   */
  ~SurroundVisionFreespace() {};
  /**
   * @brief init
   * @return result
   */
  bool Init(const std::string& sur_cfg);
  /**
   * @brief sample process
   * @return result
   */
  void Postprocess(const sim_perception::PerceptionObjects& perception_objects, FreespaceResult &freespace_result);
  FreespaceObstacle detectObstacleIntersection(const double& angle, FreespaceObstacle& det_objs);




 private:
  int net_height_;
  int net_width_;
  float freespace_dot_density_;
  bool hitObstacle = false;
};
}
#endif //SURROUND_VISION_POSTPROCESS_FREESPACE_FREESPACE_PROCESS_H_
