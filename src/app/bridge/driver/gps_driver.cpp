// Copyright 2022 The XUCONG Authors. All Rights Reserved.

#include "app/bridge/driver/gps_driver.h"

#include "common/adapters/adapter_gflags.h"
#include "common/log/Logger.h"

using namespace stoic;
using namespace stoic::app::bridge;
static const char* LOG_TAG = "main";

namespace stoic::app::bridge {
void GpsDriver::run() {
  using GpsType = sim_localization::Gps;
  GpsType gps_result;
  sim_localization::GpsUdp latest_gps_info;

  //
  static ::stoic::cm::Publisher pub_imu_ =
    nh_ptr_->advertise<GpsType, true>(FLAGS_sim_gps_topic, 1);

  Rate rate(2000);
  while (stoic::cm::ok()) {
    // read data
    recvData(latest_gps_info);
    convert2GpsMsg(latest_gps_info, gps_result);
    pub_imu_.publish<sim_localization::Gps, true>(gps_result);

    rate.sleep();
  }

}

bool GpsDriver::convert2GpsMsg(const sim_localization::GpsUdp &gps_udp, sim_localization::Gps &msg) {
  msg.header.stamp = Timer::now_us();
  msg.header.seq = ++seq_num_;
  msg.gps_info.pose_info.x = gps_udp.gps_info.pose_info.x;
  msg.gps_info.pose_info.y = gps_udp.gps_info.pose_info.y;
  msg.gps_info.pose_info.z = gps_udp.gps_info.pose_info.z;
  msg.gps_info.linear_velocity_info.x = gps_udp.gps_info.linear_velocity_info.x;
  msg.gps_info.linear_velocity_info.y = gps_udp.gps_info.linear_velocity_info.y;
  msg.gps_info.linear_velocity_info.z = gps_udp.gps_info.linear_velocity_info.z;

  LOG_INFO("simTime = %lf, simFrame = %ld\n", msg.header.stamp, msg.header.seq);
}

bool GpsDriver::init(const YAML::Node &node) {
  std::string key = "gps";
  YAML::Node result = common::YamlUtil::GetValue(node, key);
  if (!result.IsNull() && result.IsMap()) {
    bind_port_ = result["bind_port"].as<int>();
  } else {
    LOG_ERROR("Key [%s] is not found in the YAML data", key.c_str());
  }
}

}  // namespace stoic::app::bridge
