//
// Created by xucong on 23-8-7.
//

#include "app/bridge/driver/actor_control.h"
#include "common/log/Logger.h"

using namespace stoic;
using namespace stoic::app::bridge;
static const char* LOG_TAG =  "main";

namespace stoic::app::bridge {

void ActorControl::CtlCmdCallback(const std::shared_ptr<caic_control::ControlCommand> &msg) {
  int64_t current_timestamp = Timer::now_ms();

  if (msg->available == 0) {
    LOG_ERROR("Control command failed to parse!");
    latest_control_cmd.acceleration = 0.0;
    latest_control_cmd.steering_target = 0.0;
  } else {
    latest_control_cmd = *msg;
    is_control_ready = true;
  }

  // if control command coming too soon, just ignore it.
  if (current_timestamp - last_control_timestamp_ < FLAGS_min_cmd_interval * 1000) {
    LOG_ERROR("Control command comes too soon. Ignore.\n"
              " Required  topic: %s, FLAGS_min_cmd_interval[%d], actual time interval[%f].",
              FLAGS_control_topic.c_str(), FLAGS_min_cmd_interval, current_timestamp - last_control_timestamp_);
//    return;
  }

  last_control_timestamp_ = current_timestamp;
  LOG_INFO("Control sequence number: %d, Time_of_delay: %d micro seconds",
            latest_control_cmd.header.seq,
            current_timestamp - static_cast<int64_t>(latest_control_cmd.header.stamp * 1e3));

  if (std::isnan(latest_control_cmd.acceleration))
    latest_control_cmd.acceleration = 0.0;
  if (std::isnan(latest_control_cmd.steering_target))
    latest_control_cmd.steering_target = 0.0;

  LOG_INFO("===============Control Command instance===============\n"
           "  received topic is : %s , accel: %.3f, steering: %.3f",
           FLAGS_control_topic.c_str(),
           latest_control_cmd.acceleration,
           latest_control_cmd.steering_target / 8.726646 * 57.3);

}


void ActorControl::run() {
  sim_control::ControlUdp control_cmd_send;

  static stoic::cm::Subscriber sub_sim_ctl = nh_ptr_->subscribe<::caic_control::ControlCommand, true>(
        FLAGS_control_topic, 1,
        [this](auto && ctl_data) { CtlCmdCallback(std::forward<decltype(ctl_data)>(ctl_data)); });

  Rate rate(100);
  while (stoic::cm::ok()) {
    CmdMsgHandle(latest_control_cmd, control_cmd_send);

    rate.sleep();

  }
}


bool ActorControl::CmdMsgHandle(const caic_control::ControlCommand &msg, sim_control::ControlUdp &cmd_send) {
  if (!is_control_ready) {
    LOG_ERROR("Control command msg is not ready!");
    return false;
  }

  cmd_send.available = 1;
  cmd_send.acceleration = latest_control_cmd.acceleration;
  cmd_send.steering_angle = latest_control_cmd.steering_target / 100 * 8.726646;

  sendData(cmd_send);

  return true;

}

bool ActorControl::init(const YAML::Node &node) {
  //load config
//  YAML::Node config_node = YAML::LoadFile(FLAGS_udp_bridge_conf_file);
  std::string key = "actor_control";
  YAML::Node result = common::YamlUtil::GetValue(node, key);
  if (!result.IsNull() && result.IsMap()) {
    remote_ip_ = result["remote_ip"].as<std::string>();
    remote_port_ = result["remote_port"].as<int>();
  } else {
    LOG_ERROR("Key [%s] is not found in the YAML data", key.c_str());
  }
}

} // namespace stoic::app::core
