
//
// Created by xucong on 24-1-19.
//
#ifndef SIM_PERCEPTION_FRONT_VISION_FRONTVISION_DET_ALG_H_
#define SIM_PERCEPTION_FRONT_VISION_FRONTVISION_DET_ALG_H_
#include <yaml-cpp/yaml.h>
#include <boost/bind/bind.hpp>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <string>
#include <queue>
#include "sim_interface.h"
#include "ad_interface.h"
#include "common/log/Logger.h"
#include "common/time/timer.h"
#include "common/adapters/adapter_gflags.h"
#include "app/core/sim_perception/front_vision/frontvision_common/frontvision_common.h"

namespace stoic::app::core {
class FrontVisionDetectAlg {
 public:
  FrontVisionDetectAlg() { Init(); };
  ~FrontVisionDetectAlg(){};
  // interface for alg
  bool ALG_run(sim_perception::PerceptionObjects &msg);
  bool getDetectOutput(caic_perception::PerceptionObjects &output);
  bool getModelOutput(caic_perception::PerceptionObjects& output);
  bool getTrafficLtDetectOutput(DetectResultTls& output);
 private:
  void Init();
  void ConvertObjsToMsg(sim_perception::PerceptionObjects &msg, caic_perception::PerceptionObjects &output);
 public:
  std::queue<caic_perception::PerceptionObjects> detect_results_;
  std::queue<caic_perception::PerceptionObjects> model_results_;
  std::queue<DetectResultTls> trafficlght_results_;
  std::mutex mutex_detect_results_;
  std::mutex mutex_model_results_;
  std::mutex mutex_trafficlght_results_;
};
}
#endif //SIM_PERCEPTION_FRONT_VISION_FRONTVISION_DET_ALG_H_
