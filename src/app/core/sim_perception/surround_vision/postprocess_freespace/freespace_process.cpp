
//
// Copyright (c) 2024 xucong Authors. All rights reserved.
// Created by xucong on 24-3-14.
//
#include "freespace_process.h"
#include "common/config.h"
#include "common/log/Logger.h"
static const char* LOG_TAG = "freespace";

namespace stoic::app::core {
void SurroundVisionFreespace::Postprocess(const sim_perception::PerceptionObjects &perception_objects,
                                          FreespaceResult &freespace_result) {
  int startAngle = 270;
  float squareDis = 6.24;
  FreespaceObstacle det_obj(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  for (size_t j = 0; j < 1; j++) {
    det_obj.vcs_x = 10;
    det_obj.vcs_y = 10;
  }
  for (int i = 0; i < 360; ++i) {
    FreespaceObstacle detObjPoint = detectObstacleIntersection(i, det_obj);
    FreespacePoint fs_point;
    if (detObjPoint.vcs_x != std::numeric_limits<double>::max() ) {
      if (i <= 45 || i > 315) {
        fs_point.vcs_x = squareDis * std::tan(i * M_PI / 180);
        fs_point.vcs_y = squareDis;
      } else if ( i <= 135) {
        fs_point.vcs_x = squareDis;
        fs_point.vcs_y = squareDis * std::tan((90 - i) * M_PI / 180);
      } else if ( i <= 225) {
        fs_point.vcs_x = squareDis * std::tan((180 - i) * M_PI / 180);
        fs_point.vcs_y = -squareDis;
      } else if ( i <= 315) {
        fs_point.vcs_x = -squareDis;
        fs_point.vcs_y = squareDis * std::tan((i - 270) * M_PI / 180);
      } else { continue;}
      freespace_result.fs_points_.push_back(fs_point);
      }

  }
}
FreespaceObstacle SurroundVisionFreespace::detectObstacleIntersection(const double &angle,
                                                                      FreespaceObstacle &det_objs) {
  double dx = det_objs.vcs_x;
  double dy = det_objs.vcs_y;
  double obstacleDistance = std::sqrt(dx*dx + dy*dy);
  if (obstacleDistance > 6.24) {
    LOG_ERROR("[FreeSpace]: 可行驶域内无障碍物！！！");
    return FreespaceObstacle(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  } else {
    double rad = angle * (M_PI / 180);
    hitObstacle = true;
    // 返回障碍物坐标
    return FreespaceObstacle(obstacleDistance * cos(rad), obstacleDistance * sin(rad));
  }
}
bool SurroundVisionFreespace::Init(const std::string& sur_cfg) {
  YAML::Node surround_params;
  try {
    surround_params = YAML::LoadFile(sur_cfg);
  } catch (YAML::BadFile &e) {
    LOG_ERROR("[SurroundFreespace] load file error!!");
    return false;
  }
  YAML::Node freespace_params = surround_params["freespace"];
  net_height_ = freespace_params["net_height"].as<int32_t>();
  net_width_ = freespace_params["net_width"].as<int32_t>();
  // 设置可行驶区域点密度
  freespace_dot_density_ = freespace_params["freespace_dot_density"].as<float>();
};
} // namespace stoic::app::core
