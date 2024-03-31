
// Copyright 2022 The XUCONG Authors. All Rights Reserved.
#include "app/bridge/sensor/traffic_lights_sensor.h"
#include "common/adapters/adapter_gflags.h"
#include "common/log/Logger.h"
#include "common/config.h"

using namespace stoic;
using namespace stoic::app::bridge;
static const char* LOG_TAG = "main";

namespace stoic::app::bridge {
void TrafficLightSensor::run() {
  sim_ground_truth::TrafficLights traffic_lights{};
  sim_perception::PerceptionTrafficLights msg_tlights{};
  std::string config_file = std::string(getInstallRootPath()) + "/config/bridge/config.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file);
  init(config_node);
  // connect to client
  connectToUdpClient(bind_port_);
  //
  static ::stoic::cm::Publisher pub_vis =
    nh_ptr_->advertise<sim_perception::PerceptionTrafficLights, true>(FLAGS_sim_traffic_light_topic, 1);
  Rate rate(50);
  while (stoic::cm::ok()) {
    recvData(traffic_lights);
    convert2TlightsMsg(traffic_lights, msg_tlights);
    pub_vis.publish<sim_perception::PerceptionTrafficLights, true>(msg_tlights);
    rate.sleep();
  }
}
bool TrafficLightSensor::convert2TlightsMsg(const sim_ground_truth::TrafficLights &traffic_lights,
                                           sim_perception::PerceptionTrafficLights &msg) {
  // header
  msg.header.stamp = Timer::now();
  msg.header.seq = seq_num_++;
  int tl_size = 0;
  uint32_t noElements = sizeof(traffic_lights.traffic_light) / sizeof(sim_ground_truth::TrafficLight);
  msg.size = noElements;
  LOG_INFO("====ProcVisionInfo: got start of frame===");
  LOG_INFO("simTime = %.3lf, simFrame = %ld", traffic_lights.header.simTime, traffic_lights.header.frameNo );
  for (size_t i = 0; i < noElements; i++) {
    sim_ground_truth::TrafficLight tl_data = traffic_lights.traffic_light[i];
    if (tl_data.id == 0) {
      LOG_ERROR("接收红绿灯失败");
      continue;
    }
    msg.traffic_lights[i].id[0] = '0';
    msg.traffic_lights[i].color = tl_color_[tl_data.color];
    msg.traffic_lights[i].show_type = tl_show_type_[tl_data.show_type];
    msg.traffic_lights[i].confidence = 1.0;
    msg.traffic_lights[i].blink = false;
    msg.traffic_lights[i].remaining_time = tl_data.remaining_time;
    tl_size++;
  }
  msg.size = tl_size;
  LOG_ERROR("[Vision] 接收红绿灯数量：%d", msg.size);
  LOG_INFO("====ProcVisionInfo: got end of frame===\n");
}
bool TrafficLightSensor::init(const YAML::Node &node) {
  std::string key = "light";
  YAML::Node result = common::YamlUtil::GetValue(node, key);
  if (!result.IsNull() && result.IsMap()) {
    bind_port_ = result["bind_port"].as<int>();
  } else {
    LOG_ERROR("Key [%s] is not found in the YAML data", key.c_str());
  }
}
}  // namespace stoic::app::bridge
