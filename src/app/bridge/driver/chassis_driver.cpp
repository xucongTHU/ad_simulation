// Copyright 2022 The XUCONG Authors. All Rights Reserved.

#include "app/bridge/driver/chassis_driver.h"

#include "common/adapters/adapter_gflags.h"
#include "common/log/Logger.h"

using namespace stoic;
using namespace stoic::app::bridge;
static const char* LOG_TAG = "main";

namespace stoic::app::bridge {

void ChassisDriver::run() {
  sim_chassis::Chassis chassis_result;
  sim_chassis::ChassisUdp latest_chassis_info;

  // publisher
  static ::stoic::cm::Publisher pub_sim_chassis =
      nh_ptr_->advertise<sim_chassis::Chassis, true>(FLAGS_sim_chassis_topic, 1);

  Rate rate(2000);
  while (stoic::cm::ok()) {
    // read data
    recvData(latest_chassis_info);
    convert2SimChassis(latest_chassis_info, chassis_result);
    pub_sim_chassis.publish<sim_chassis::Chassis, true>(chassis_result);

    rate.sleep();
  }

}

bool ChassisDriver::convert2SimChassis(const sim_chassis::ChassisUdp &chassis_udp, sim_chassis::Chassis &msg) {
  msg.header.stamp = Timer::now_us();
  msg.header.seq = ++seq_num_;
  msg.chassis_motion.vehicle_speed = chassis_udp.chassis_motion.vehicle_speed;
  msg.steering.steering_wheel_info.angle = chassis_udp.steering.steering_wheel_info.angle;
  msg.steering.steering_wheel_info.speed = chassis_udp.steering.steering_wheel_info.speed;
  msg.wheels.wheel_speed_info.fl = chassis_udp.wheels.wheel_speed_info.fl;
  msg.wheels.wheel_speed_info.fr = chassis_udp.wheels.wheel_speed_info.fr;
  msg.wheels.wheel_speed_info.rl = chassis_udp.wheels.wheel_speed_info.rl;
  msg.wheels.wheel_speed_info.rr = chassis_udp.wheels.wheel_speed_info.rr;
  msg.powertrain.gear_state = sim_std::GearState::DRIVE;
  msg.chassis_motion.driving_direction = sim_chassis::DrivingDirection::FORWARD;
  switch (chassis_udp.gear_state)
  {
    case -1:
      msg.powertrain.gear_state = sim_std::GearState::REVERSE;
    case 0:
      msg.powertrain.gear_state = sim_std::GearState::NEUTRAL;
      break;
    default:
      msg.powertrain.gear_state = sim_std::GearState::DRIVE;
      break;
  }

  LOG_INFO("simTime = %lf, simFrame = %ld\n", msg.header.stamp, msg.header.seq);


}

bool ChassisDriver::init(const YAML::Node &node) {
  std::string key = "chassis";
  YAML::Node result = common::YamlUtil::GetValue(node, key);
  if (!result.IsNull() && result.IsMap()) {
    bind_port_ = result["bind_port"].as<int>();
  } else {
    LOG_ERROR("Key [%s] is not found in the YAML data", key.c_str());
  }
}

}  // namespace stoic::app::bridge
