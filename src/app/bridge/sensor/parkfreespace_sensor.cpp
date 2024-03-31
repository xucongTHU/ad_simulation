
// Copyright 2022 The XUCONG Authors. All Rights Reserved.
#include "app/bridge/sensor/parkfreespace_sensor.h"
#include "common/adapters/adapter_gflags.h"
#include "common/log/Logger.h"
#include "common/config.h"

using namespace stoic;
using namespace stoic::app::bridge;
static const char* LOG_TAG = "main";

namespace stoic::app::bridge {
void ParkFreespaceSensor::run() {
  sim_ground_truth::ParkingSpaces parking_spaces{};
  sim_perception::PerceptionSurround msg_surround{};
  std::string config_file = std::string(getInstallRootPath()) + "/config/bridge/config.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file);
  init(config_node);
  // connect to client
  connectToUdpClient(bind_port_);
  //
  static ::stoic::cm::Publisher pub_vis =
    nh_ptr_->advertise<sim_perception::PerceptionObjects, true>("/stoic/surround/parkspace", 1);
  Rate rate(50);
  while (stoic::cm::ok()) {
    recvData(parking_spaces);
    convert2SurroundMsg(parking_spaces, msg_surround);
    pub_vis.publish<sim_perception::PerceptionSurround, true>(msg_surround);
    rate.sleep();
  }
}
bool ParkFreespaceSensor::convert2SurroundMsg(const sim_ground_truth::ParkingSpaces &parking_spaces,
                                              sim_perception::PerceptionSurround &msg) {
  // header
  msg.header.stamp = Timer::now();
  msg.header.seq = seq_num_++;
  int ps_size = 0;
  uint32_t noElements = sizeof(parking_spaces.parking_space) / sizeof(sim_ground_truth::ParkingSpace);
  msg.parkingspace_result.parking_spaces_size = noElements;
  LOG_INFO("[ParkFreespaceSensor]: got start of frame ...");
  LOG_INFO("simTime = %.3lf, simFrame = %ld", parking_spaces.header.simTime, parking_spaces.header.frameNo );
  for (uint32_t i = 0; i < noElements; i++) {
    sim_ground_truth::ParkingSpace ps_data = parking_spaces.parking_space[i];
    if (ps_data.type == 0) {
      LOG_ERROR("未检测到车位！！！");
      continue;
    }
    msg.parkingspace_result.parking_spaces[i].track_id = ps_data.id;
    msg.parkingspace_result.parking_spaces[i].space_vcs.parkingspace_left_up_point =
        ps_data.park_left_up_point;
    msg.parkingspace_result.parking_spaces[i].space_vcs.parkingspace_left_down_point =
        ps_data.park_left_down_point;
    msg.parkingspace_result.parking_spaces[i].space_vcs.parkingspace_right_up_point =
        ps_data.park_right_up_point;
    msg.parkingspace_result.parking_spaces[i].space_vcs.parkingspace_right_down_point =
        ps_data.park_right_down_point;
    msg.parkingspace_result.parking_spaces[i].space_vcs.angle_left = ps_data.angleLeft;
    msg.parkingspace_result.parking_spaces[i].space_vcs.angle_right = ps_data.angleRight;
    msg.parkingspace_result.parking_spaces[i].type = ps_data.type;
    ps_size++;
    LOG_ERROR("[ParkFreespaceSensor] type: %d", ps_data.type);
    LOG_ERROR("[ParkingSpace]: 角点坐标left-up（%.2f, %.2f）, 角点坐标left-down（%.2f, %.2f）, "
              "角点坐标right-up（%.2f, %.2f）, 角点坐标right-down（%.2f, %.2f）",
              ps_data.park_left_up_point.x, ps_data.park_left_up_point.y,
              ps_data.park_left_down_point.x, ps_data.park_left_down_point.y,
              ps_data.park_right_up_point.x, ps_data.park_right_up_point.y,
              ps_data.park_right_down_point.x, ps_data.park_right_down_point.y);
  }
  msg.parkingspace_result.parking_spaces_size = ps_size;
  LOG_ERROR("[ParkFreespaceSensor] 检测车位数量：%d", msg.parkingspace_result.parking_spaces_size);
  LOG_INFO("[ParkFreespaceSensor]: got end of frame ...\n");
}
bool ParkFreespaceSensor::init(const YAML::Node &node) {
  std::string key = "parking";
  YAML::Node result = common::YamlUtil::GetValue(node, key);
  if (!result.IsNull() && result.IsMap()) {
    bind_port_ = result["bind_port"].as<int>();
  } else {
    LOG_ERROR("Key [%s] is not found in the YAML data", key.c_str());
  }
}
}  // namespace stoic::app::bridge
