
//
// Created by xucong on 23-11-10.
//
#pragma once
#include "simulator/vtd_integration/vtd_dependencies/pattern/workflow.h"
#include "simulator/vtd_integration/front_vision/app/front_vision_model.h"
#include "simulator/vtd_integration/parkingspace_freespace/parkspace_freespace_model.h"

namespace stoic::simulator {
class WorkflowLoader {
 public:
  WorkflowLoader(const std::string& deployment) : deployment_(deployment) {}
  void init(pattern::Workflow* wf) {
    thread_pool_.reset(new ThreadPool(3));
    // front vision
    front_vision_model_  = std::make_shared<FrontVisionModel>("front vision dector", "FVIS:ALL", "Vision");
    parkspace_freespace_model_ = std::make_shared<ParkFreespaceModel>("parkspace_freespace dector", "PARK:ALL", "Vision");
    if (deployment_.find("FVIS") != std::string::npos ||
        deployment_.find("ALL") != std::string::npos) {
      thread_pool_->enqueue([this] { this->front_vision_model_->run(); });
    }
    if (deployment_.find("PARK") != std::string::npos ||
        deployment_.find("ALL") != std::string::npos) {
      thread_pool_->enqueue([this] { this->parkspace_freespace_model_->run(); });
    }
  }
 private:
  std::string deployment_;
  std::unique_ptr<ThreadPool> thread_pool_;
  std::shared_ptr<FrontVisionModel> front_vision_model_;
  std::shared_ptr<ParkFreespaceModel> parkspace_freespace_model_;
};
} // namespace stoic::simulator
