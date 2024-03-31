
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
 * @class perc traffic lights
 */
class TrafficLightSensor : public pattern::Task<TrafficLightSensor>,
                        public UdpSocketServer<sim_ground_truth::TrafficLights>{
 public:
  template<typename... OptionArgs>
  TrafficLightSensor(stoic::cm::NodeHandle& nh, OptionArgs&&... option_args) :
    Task(nh, std::forward<OptionArgs>(option_args)...) {
    nh_ptr_ = &nh;
  }
  TrafficLightSensor(stoic::cm::NodeHandle& nh, const pattern::TaskOptions& task_options) : Task(nh, task_options) {
    nh_ptr_ = &nh;
  }
  ~TrafficLightSensor() = default;
  void run() override;
 private:
  bool init(const YAML::Node& node);
  bool convert2TlightsMsg(const sim_ground_truth::TrafficLights&, sim_perception::PerceptionTrafficLights&);
 private:
  stoic::cm::NodeHandle* nh_ptr_;
  int32_t seq_num_ = 0;
  int32_t bind_port_;
  std::unordered_map<int, sim_perception::TrafficLight::ShowType> tl_show_type_ = {
      { 0, sim_perception::TrafficLight::ShowType::UNKNOWN },
      { 10,  sim_perception::TrafficLight::ShowType::LEFT_ARROW },
      { 30,  sim_perception::TrafficLight::ShowType::FORWARD_ARROW },
      { 20,  sim_perception::TrafficLight::ShowType::RIGHT_ARROW },
      { 80,  sim_perception::TrafficLight::ShowType::TURN_AROUND }};
  std::unordered_map<int, sim_perception::TrafficLight::Color> tl_color_ = {
      { 0, sim_perception::TrafficLight::Color::UNKNOWN },
      { 1,  sim_perception::TrafficLight::Color::RED },
      { 2,  sim_perception::TrafficLight::Color::YELLOW },
      { 3,  sim_perception::TrafficLight::Color::GREEN }};
};
}  // namespace stoic::app::bridge
WORKFLOW_ADD_TASK(::stoic::app::bridge::TrafficLightSensor)
