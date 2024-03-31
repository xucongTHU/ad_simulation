// Copyright 2022 The XUCONG Authors. All Rights Reserved.

#include "app/bridge/driver/imu_driver.h"

#include "common/adapters/adapter_gflags.h"
#include "common/log/Logger.h"

using namespace stoic;
using namespace stoic::app::bridge;
static const char* LOG_TAG = "main";

namespace stoic::app::bridge {
void ImuDriver::run() {
  using ImuType = sim_localization::Imu;
  ImuType imu_result;
  sim_localization::ImuUdp latest_imu_info;

  //
  static ::stoic::cm::Publisher pub_imu_ =
    nh_ptr_->advertise<ImuType, true>(FLAGS_sim_imu_topic, 1);

  Rate rate(2000);
  while (stoic::cm::ok()) {
    recvData(latest_imu_info);
    convert2ImuMsg(latest_imu_info, imu_result);
    pub_imu_.publish<sim_localization::Imu, true>(imu_result);

    rate.sleep();
  }

}

bool ImuDriver::convert2ImuMsg(const sim_localization::ImuUdp &imu_udp, sim_localization::Imu &msg) {
  msg.header.stamp = Timer::now_us();
  msg.header.seq = ++seq_num_;
  msg.imu_info.linear_acceleration_vrf_info.x = imu_udp.imu_info.linear_acceleration_vrf_info.x;
  msg.imu_info.linear_acceleration_vrf_info.y = imu_udp.imu_info.linear_acceleration_vrf_info.y;
  msg.imu_info.linear_acceleration_vrf_info.z = imu_udp.imu_info.linear_acceleration_vrf_info.z;
  msg.imu_info.angular_velocity_vrf_info.x = imu_udp.imu_info.angular_velocity_vrf_info.x;
  msg.imu_info.angular_velocity_vrf_info.y = imu_udp.imu_info.angular_velocity_vrf_info.y;
  msg.imu_info.angular_velocity_vrf_info.z = imu_udp.imu_info.angular_velocity_vrf_info.z;
  msg.imu_info.angle.roll = imu_udp.imu_info.angle.roll;
  msg.imu_info.angle.pitch = imu_udp.imu_info.angle.pitch;
  msg.imu_info.angle.yaw = imu_udp.imu_info.angle.yaw;

  LOG_INFO("simTime = %lf, simFrame = %ld\n", msg.header.stamp, msg.header.seq);
}

bool ImuDriver::init(const YAML::Node &node) {
  std::string key = "imu";
  YAML::Node result = common::YamlUtil::GetValue(node, key);
  if (!result.IsNull() && result.IsMap()) {
    bind_port_ = result["bind_port"].as<int>();
  } else {
    LOG_ERROR("Key [%s] is not found in the YAML data", key.c_str());
  }
}

}  // namespace stoic::app::bridge
