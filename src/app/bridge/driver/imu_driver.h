// Copyright 2022 The XUCONG Authors. All Rights Reserved.


#include "cm/cm.h"
#include "pattern/task.hpp"
#include "sim_interface.h"

#include "common/time/timer.h"
#include "common/util/yaml_util.h"
#include "common/network/udpsocket_server.h"
#include "app/bridge/bridge_dependencies/common/bridge_gflags.h"


namespace stoic::app::bridge {

/**
 * @class ImuDriver
 * @brief base class for imu receiver
 */
class ImuDriver : public pattern::Task<ImuDriver>,
                  public UdpSocketServer<sim_localization::ImuUdp> {
 public:
  template<typename... OptionArgs>
  ImuDriver(stoic::cm::NodeHandle& nh, OptionArgs&&... option_args) :
  Task(nh, std::forward<OptionArgs>(option_args)...) {

    nh_ptr_ = &nh;
    YAML::Node config_node = YAML::LoadFile(FLAGS_udp_bridge_conf_file);
    init(config_node);
    // connect to client
    connectToUdpClient(bind_port_);

  }

  ImuDriver(stoic::cm::NodeHandle& nh, const pattern::TaskOptions& task_options) : Task(nh, task_options) {
    nh_ptr_ = &nh;
  }

  ~ImuDriver() = default;

  void run() override;

 private:
  bool convert2ImuMsg(const sim_localization::ImuUdp&, sim_localization::Imu&);
  bool init(const YAML::Node &node);

 private:
  stoic::cm::NodeHandle* nh_ptr_;
  int32_t seq_num_;
  int32_t bind_port_;

};

}  // namespace stoic::app::bridge

WORKFLOW_ADD_TASK(::stoic::app::bridge::ImuDriver)
