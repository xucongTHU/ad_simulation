//
// Created by xucong on 23-8-10.
//

#pragma once

#include "cm/cm.h"
#include "pattern/task.hpp"

#include <yaml-cpp/yaml.h>
#include "sim_interface.h"
#include "ad_interface.h"
#include "common/time/timer.h"
#include "common/util/yaml_util.h"
#include "common/network/udpsocket_client.h"
#include "common/adapters/adapter_gflags.h"
#include "app/bridge/bridge_dependencies/common/bridge_gflags.h"

namespace stoic::app::bridge {

/**
 * @class SimControl
 * @brief A module subscribe subscribe stoic control command,
 * and transform cmd to carsim control with bridge module.
 */
class ActorControl : public pattern::Task<ActorControl>,
                     public UdpSocketClient<sim_control::ControlUdp> {
 public:
  template<typename... OptionArgs>
  ActorControl(stoic::cm::NodeHandle& nh, OptionArgs&&... option_args) :
      Task(nh, std::forward<OptionArgs>(option_args)...) {

    nh_ptr_ = &nh;
    YAML::Node config_node = YAML::LoadFile(FLAGS_udp_bridge_conf_file);
    init(config_node);
    // connect to server
    connectToUdpServer(remote_ip_, remote_port_);

    // static stoic::cm::Subscriber sub_sim_ctl = nh_ptr_->subscribe<::caic_control::ControlCommand, true>(
    //     FLAGS_control_topic, 1,
    //     [this](auto && ctl_data) { CtlCmdCallback(std::forward<decltype(ctl_data)>(ctl_data)); });


  }

  ActorControl(stoic::cm::NodeHandle& nh, const pattern::TaskOptions& task_options) : Task(nh, task_options) {
    nh_ptr_ = &nh;
  }

  ~ActorControl() = default;

  void CtlCmdCallback(const std::shared_ptr<caic_control::ControlCommand>& msg);

  void run() override;

 private:
  bool CmdMsgHandle(const caic_control::ControlCommand&, sim_control::ControlUdp&);
  bool init(const YAML::Node &node);

 private:
  stoic::cm::NodeHandle* nh_ptr_;
  caic_control::ControlCommand latest_control_cmd;

  double acceleration;
  double steering_angle;
  int64_t last_control_timestamp_ = 0;
  int64_t seq_num = 0;
  bool is_control_ready = false;
  std::string remote_ip_;
  int32_t remote_port_;


};

} // namespace stoic::app::core

WORKFLOW_ADD_TASK(::stoic::app::bridge::ActorControl)

