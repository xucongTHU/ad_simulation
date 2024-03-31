
//
// Created by xucong on 24-1-3.
//
#ifndef SIM_PERCEPTION_FRONT_VISION_DETECTOR_H_
#define SIM_PERCEPTION_FRONT_VISION_DETECTOR_H_
#include "cm/cm.h"
#include "pattern/task.hpp"
#include "sim_interface.h"
#include "ad_interface.h"
#include "common/log/Logger.h"
#include "common/time/timer.h"
#include "pattern/ThreadPool.h"
#include "common/adapters/adapter_gflags.h"
#include "app/core/sim_perception/front_vision/frontvision_common/frontvision_common.h"
#include "app/core/sim_perception/front_vision/frontvision_common/frontvision_det_alg.h"
namespace stoic::app::core {
class FrontVisionDetector : public pattern::Task<FrontVisionDetector> {
 public:
  template <typename... OptionArgs>
  FrontVisionDetector(stoic::cm::NodeHandle& nh, OptionArgs&&... option_args) :
      Task(nh, std::forward<OptionArgs>(option_args)...) {
    nh_ptr_ = &nh;
  }
  FrontVisionDetector(stoic::cm::NodeHandle& nh, const pattern::TaskOptions& task_options) : Task(nh, task_options) {
    nh_ptr_ = &nh;
  }
  ~FrontVisionDetector() = default;
  void objCallback(const std::shared_ptr<sim_perception::PerceptionObjects>& msg);
  void tlCallback(const std::shared_ptr<sim_perception::PerceptionTrafficLights>& msg);
  void PoseCallback(const std::shared_ptr<caic_localization::LocalizationEstimation>& msg);
  void run() override;
  void ObjectsPublish();
  void TrafficlightsPublish();
 private:
  void Init();
  void detect(sim_perception::PerceptionObjects& sensor_img);
  void ConvertTl2CAICMsg(const sim_perception::PerceptionTrafficLights& output,
                         std::shared_ptr<caic_perception::PerceptionTrafficLights>& msg_caic);
  void ConvertObjsToMsg(const sim_perception::PerceptionObjects &output,
                        std::shared_ptr<caic_perception::PerceptionObjects> &perc_objs);
 private:
  stoic::cm::NodeHandle* nh_ptr_;
  std::shared_ptr<FrontVisionDetectAlg> detect_alg_;
  ThreadPool thread_pool_{1};
  sim_perception::PerceptionObjects perception_objects_;
  sim_perception::PerceptionTrafficLights perception_traffic_lights_;
  std::shared_ptr<caic_localization::LocalizationEstimation> pose_manager;

  bool is_perc_ready = false;
  bool is_traffic_light_ready = false;
  bool is_loc_ready = false;
  std::unordered_map<sim_perception::TrafficLight::ShowType, caic_perception::TrafficLight::ShowType> tl_show_type_ = {
      { sim_perception::TrafficLight::ShowType::UNKNOWN, caic_perception::TrafficLight::ShowType::UNKNOWN },
      { sim_perception::TrafficLight::ShowType::LEFT_ARROW,  caic_perception::TrafficLight::ShowType::LEFT_ARROW },
      { sim_perception::TrafficLight::ShowType::FORWARD_ARROW,  caic_perception::TrafficLight::ShowType::FORWARD_ARROW },
      { sim_perception::TrafficLight::ShowType::RIGHT_ARROW,  caic_perception::TrafficLight::ShowType::RIGHT_ARROW },
      { sim_perception::TrafficLight::ShowType::TURN_AROUND,  caic_perception::TrafficLight::ShowType::TURN_AROUND }};
  std::unordered_map<sim_perception::TrafficLight::Color, caic_perception::TrafficLight::Color> tl_color_ = {
      { sim_perception::TrafficLight::Color::UNKNOWN, caic_perception::TrafficLight::Color::UNKNOWN },
      { sim_perception::TrafficLight::Color::RED,  caic_perception::TrafficLight::Color::RED },
      { sim_perception::TrafficLight::Color::YELLOW,  caic_perception::TrafficLight::Color::YELLOW },
      { sim_perception::TrafficLight::Color::GREEN,  caic_perception::TrafficLight::Color::GREEN },
      { sim_perception::TrafficLight::Color::BLACK,  caic_perception::TrafficLight::Color::BLACK }};
};
} //namespace stoic::app::core
WORKFLOW_ADD_TASK(::stoic::app::core::FrontVisionDetector)
#endif //SIM_PERCEPTION_FRONT_VISION_DETECTOR_H_
