
// Copyright 2022 The XUCONG Authors. All Rights Reserved.

#include "cm/cm.h"
#include "pattern/task.hpp"
#include <yaml-cpp/yaml.h>
#include "sim_interface.h"
#include "common/time/timer.h"
#include "common/util/yaml_util.h"
#include "common/network/udpsocket_server.hpp"
#include "app/bridge/bridge_dependencies/common/bridge_gflags.h"
namespace stoic::app::bridge {
/**
 * @class perc objects
 */
class ObjectSensor : public pattern::Task<ObjectSensor>,
                     public UdpSocketServer<sim_ground_truth::FrontVisionObjects>{
 public:
  template<typename... OptionArgs>
  ObjectSensor(stoic::cm::NodeHandle& nh, OptionArgs&&... option_args) :
    Task(nh, std::forward<OptionArgs>(option_args)...) {
    nh_ptr_ = &nh;
  }
  ObjectSensor(stoic::cm::NodeHandle& nh, const pattern::TaskOptions& task_options) : Task(nh, task_options) {
    nh_ptr_ = &nh;
  }
  ~ObjectSensor() = default;
  void run() override;
 private:
  bool init(const YAML::Node& node);
  bool convert2ObjMsg(const sim_ground_truth::FrontVisionObjects&, sim_perception::PerceptionObjects&);
 private:
  stoic::cm::NodeHandle* nh_ptr_;
  int32_t seq_num_ = 0;
  int32_t bind_port_;
};
}  // namespace stoic::app::bridge
WORKFLOW_ADD_TASK(::stoic::app::bridge::ObjectSensor)
