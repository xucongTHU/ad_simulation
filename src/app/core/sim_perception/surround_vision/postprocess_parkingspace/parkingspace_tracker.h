
//
// Copyright (c) 2024 xucong Authors. All rights reserved.
// Created by xucong on 24-3-13.
//
#ifndef SURROUND_VISION_PARKINGSPACE_TRAKCER_H_
#define SURROUND_VISION_PARKINGSPACE_TRAKCER_H_
#pragma once
#include <atomic>
#include <memory>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Eigen>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include "manif/manif.h"
#include "proto/geometry/geometry.pb.h"
#include "proto/config/config_sensor.pb.h"
#include "app/core/sim_perception/surround_vision/common/surround_common.h"
namespace stoic::app::core {
class ParkingSpaceTracker {
 public:
  ParkingSpaceTracker(const std::string &cfg, const std::string &sensor_cfg);
  ~ParkingSpaceTracker();
  bool Init(const std::string &cfg, const std::string &sensor_cfg);
  manif::SE3d getSE3(const stoic::cm::proto::geometry::Transformationd &tf);
  void ipm2map(manif::SE3d TX_I_M, cv::Point2f &pt, Eigen::Vector3d &P_M);
  void ipm2map(manif::SE3d TX_I_M, std::vector<cv::Point2f> &pt,
               std::vector<Eigen::Vector3d> &P_M);
  void map2vcs(manif::SE3d TX_I_M, std::vector<Eigen::Vector3d> &P_M,
               std::vector<Eigen::Vector3d> &P_V);
  void vcs2ipm(std::vector<Eigen::Vector3d> &P_V, std::vector<cv::Point2f> &pt);
  float get_length_by_type(int type, float corner_dist);
  int get_ipm2vcs_xdiff() { return x_diff_; }
  int get_ipm2vcs_ydiff() { return y_diff_; }
  float get_ipm2vcs_ratio() { return ratio_; }
 private:
  int image_width_;
  int image_height_;
  int vehicle_width_;
  int vehicle_height_;
  int ipm_width_;
  int ipm_height_;
  float ratio_;
  int x_offset_;
  float angle_range_;
  std::string sensor_cfg_;
  std::string save_dir_;
  bool save_ipm_;
  bool use_angle_;
  bool debug_;
  int step_cnt_;
  bool dynamic_table_ = true;
  std::vector<std::string> topics_;
  std::string front_topic_name_;
  std::string rear_topic_name_;
  std::string left_topic_name_;
  std::string right_topic_name_;
  bool use_sensor_wh_;
  int x_diff_;
  int y_diff_;
  int down_offset_;
  manif::SE3d TX_V_I_;
  manif::SE3d TX_S_V_;
  std::unordered_map<std::string, manif::SE3d> TX_Ci_C_;
  bool inited_ = false;
};
}

#endif //SURROUND_VISION_PARKINGSPACE_TRAKCER_H_
