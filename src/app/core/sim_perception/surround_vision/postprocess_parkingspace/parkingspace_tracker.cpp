
//
// Copyright (c) 2024 xucong Authors. All rights reserved.
// Created by xucong on 24-3-13.
//
#include "parkingspace_tracker.h"
#include "common/config.h"
#include "proto/proto_utils.h"

namespace stoic::app::core {
const float PARKINGSPACE_DIST_THRESHOLD = 2.0f;
const float PS_DELETE_DIST_THRESHOLD = 20.0f;
const int PS_DELETE_LOST_THRESHOLD = 20;
/**
 * @brief 构造函数
 * @return none
 */
ParkingSpaceTracker::ParkingSpaceTracker(const std::string &cfg,
                                         const std::string &sensor_cfg) {
  if (!Init(cfg, sensor_cfg)) {
    LOG_ERROR("[SurroundParkingSpace] load cfg error, cfg = %s", cfg.c_str());
    return;
  }
}
/**
 * @brief 析构
 * @return none
 */
ParkingSpaceTracker::~ParkingSpaceTracker() {}
/**
 * @brief ipm图像到世界的转换
 * @param TX_I_M imu到地图的转换
 * @param pt ipm图像的一组点
 * @param P_M 世界坐标系下的一组点（PS：暂时不考虑高度的正确性）
 * @return void
 */
void ParkingSpaceTracker::ipm2map(manif::SE3d TX_I_M, std::vector<cv::Point2f> &pt,
                                  std::vector<Eigen::Vector3d> &P_M) {
  P_M.clear();
  P_M.resize(pt.size());
#pragma omp parallel for num_threads(4)
  for (size_t i = 0; i < pt.size(); ++i) {
    ipm2map(TX_I_M, pt[i], P_M[i]);
  }
}
/**
 * @brief ipm图像到世界的转换
 * @param TX_I_M imu到地图的转换
 * @param pt ipm图像的点
 * @param P_M 世界坐标系下的点（PS：暂时不考虑高度的正确性）
 * @return void
 */
void ParkingSpaceTracker::ipm2map(manif::SE3d TX_I_M, cv::Point2f &pt,
                                  Eigen::Vector3d &P_M) {
  float vcs_x = (-pt.y + y_diff_) / ratio_;
  float vcs_y = (-pt.x + x_diff_) / ratio_;
  Eigen::Vector3d P_V = { static_cast<double>(vcs_x),
                          static_cast<double>(vcs_y),
                          0.0 };
  manif::SE3d TX_V_M = TX_I_M * TX_V_I_;
  P_M = TX_V_M.act(P_V);
}
/**
 * @brief 世界到vcs
 * @param TX_I_M imu到地图的转换
 * @param P_M 世界坐标系下的一组点（PS：暂时不考虑高度的正确性）
 * @param P_V vcs
 * @return void
 */
void ParkingSpaceTracker::map2vcs(manif::SE3d TX_I_M,
                                  std::vector<Eigen::Vector3d> &P_M,
                                  std::vector<Eigen::Vector3d> &P_V) {
  manif::SE3d TX_M_V = TX_V_I_.inverse() * TX_I_M.inverse();
  P_V.resize(P_M.size());
#pragma omp parallel for num_threads(4)
  for (size_t i = 0; i < P_M.size(); ++i) {
    P_V[i] = TX_M_V.act(P_M[i]);
  }
}
/**
 * @brief vcs到ipm
 * @param TX_I_M imu到地图的转换
 * @param P_V vcs
 * @param pt ipm
 * @return void
 */
void ParkingSpaceTracker::vcs2ipm(std::vector<Eigen::Vector3d> &P_V,
                                  std::vector<cv::Point2f> &pt) {
  pt.resize(P_V.size());
#pragma omp parallel for num_threads(4)
  for (size_t i = 0; i < P_V.size(); ++i) {
    pt[i].x = -static_cast<float>(P_V[i][1]) * ratio_ + x_diff_;
    pt[i].y = -static_cast<float>(P_V[i][0]) * ratio_ + y_diff_;
  }
}
/**
 * @brief 根据类型返回默认车位长度
 * @param type imu到地图的转换
 * @param corner_dist 两个角点之间的距离
 * @return 车位长度
 */
float ParkingSpaceTracker::get_length_by_type(int type, float corner_dist) {
  /* type = 0 背景 | type = 1 垂直车位 | type = 2 水平车位 | type = 3 斜向车位 */
  if (type == 2 || corner_dist > PARKINGSPACE_WIDTH * 2.0f) {
    return PARKINGSPACE_WIDTH;
  } else {
    return PARKINGSPACE_HEIGHT;
  }
}
manif::SE3d ParkingSpaceTracker::getSE3(const stoic::cm::proto::geometry::Transformationd& tf) {
  Eigen::Vector3d translation(tf.translation().x(), tf.translation().y(), tf.translation().z());
  Eigen::Quaterniond quaternion(tf.quaternion().w(), tf.quaternion().x(), tf.quaternion().y(),
                                tf.quaternion().z());
  quaternion.normalize();
  return manif::SE3d(translation, quaternion);
}
bool ParkingSpaceTracker::Init(const std::string &cfg, const std::string &sensor_cfg) {
  YAML::Node IPM_params;
  try {
    IPM_params = YAML::LoadFile(cfg);
  } catch (YAML::BadFile &e) {
    LOG_ERROR("[SurroundParkingSpace] load file error!!");
    return false;
  }
  vehicle_width_ = IPM_params["vehicle_width"].as<int>();
  vehicle_height_ = IPM_params["vehicle_height"].as<int>();
  ipm_width_ = IPM_params["ipm_width"].as<int>();
  ipm_height_ = IPM_params["ipm_height"].as<int>();
  ratio_ = IPM_params["ratio"].as<float>();
  rear_topic_name_ = IPM_params["rear_fiseye_name"].as<std::string>();
  stoic::cm::proto::config::ConfigSensor config_sensor;
  if (!stoic::cm::proto::loadFromFile(sensor_cfg, &config_sensor)) {
    LOG_ERROR("[SurroundParkingSpace] Load sensor config failed.");
    return false;
  }
  TX_S_V_ = getSE3(config_sensor.tx_s_v());
  std::string sensor_name = "front_fisheye";  // 前视鱼眼相机
  for (const stoic::cm::proto::config::ConfigCamera& config_camera : config_sensor.cameras()) {
    TX_Ci_C_[sensor_name] = getSE3(config_camera.tx_ci_c());
  }
  auto TX_Ci_V = TX_S_V_ * TX_Ci_C_[sensor_name];
  double x_ci_v = TX_Ci_V.translation()[0] * ratio_;
  down_offset_ = vehicle_height_ / 2 + static_cast<int>(x_ci_v);
  x_diff_ = ipm_width_ / 2;
  y_diff_ = ipm_height_ / 2 + down_offset_;
  return true;
}
}
