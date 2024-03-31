
//
// Created by xucong on 24-1-10.
//
#pragma once
#include "pattern/workflow.hpp"
#include "pattern/ThreadPool.h"
//==============driver==============//
#include "app/bridge/driver/chassis_driver.h"
#include "app/bridge/driver/gps_driver.h"
#include "app/bridge/driver/imu_driver.h"
#include "app/bridge/actor_control/actor_control.h"
#include "app/bridge/sensor/lane_line_sensor.h"
#include "app/bridge/sensor/object_sensor.h"
#include "app/bridge/sensor/traffic_lights_sensor.h"
#include "app/bridge/sensor/parkfreespace_sensor.h"
//==============canbus==============//
#include "app/core/sim_canbus/canbus_core.h"
//===========localization===========//
#include "app/core/sim_msf_localization/app/msf_loccaization_core.h"
//============perception============//
#include "app/core/sim_perception/front_vision/lane_line.h"
#include "app/core/sim_perception/front_vision/front_vision_detector.h"
#include "app/core/sim_perception/surround_vision/surround_vision_detector.h"
using namespace stoic;
using namespace stoic::app;
using namespace stoic::app::bridge;
using namespace stoic::app::core;
namespace stoic::app {
class WorkflowLoader {
 public:
  WorkflowLoader(stoic::cm::NodeHandle &nh, const std::string &deployment) : nh_(nh), deployment_(deployment) {}
  void init(pattern::Workflow *wf) {
    thread_pool_.reset(new ThreadPool(15));
    sim_canbus_core_ = std::make_shared<CanbusCore>(nh_, "sim canbus", "CAN:CHS:ALL", "Chassis", "sim canbus");
    sim_msf_localization_core_ = std::make_shared<MsfLocalizationCore>(nh_, "sim msf loc", "LOC:CHS:ALL", "Localization", "sim loc");
    if (deployment_.find("LOC") != std::string::npos ||
        deployment_.find("CHS") != std::string::npos ||
        deployment_.find("ALL") != std::string::npos) {
      thread_pool_->enqueue([this] { this->sim_msf_localization_core_->run(); });
    }
    if (deployment_.find("CAN") != std::string::npos ||
        deployment_.find("CHS") != std::string::npos ||
        deployment_.find("ALL") != std::string::npos) {
      thread_pool_->enqueue([this] { this->sim_canbus_core_->run(); });
    }
    // ================= [driver] ================= //
    create_driver_workflow(wf);
  
    // ================== [perc] ================== //
    create_perc_workflow(wf);

  }
  void create_driver_workflow(pattern::Workflow* wf) {
    actor_control_ = std::make_shared<ActorControl>(nh_, "actor control", "ACT:DRI:ALL", "ACTL", "actl");
    chassis_driver_ = std::make_shared<ChassisDriver>(nh_, "chassis driver", "CHA:DRI:ALL", "Chassis", "driver");
    gps_driver_ = std::make_shared<GpsDriver>(nh_, "gps driver", "GPS:DRI:ALL", "Gps", "driver");
    imu_driver_ = std::make_shared<ImuDriver>(nh_, "imu driver", "IMU:DRI:ALL", "Imu", "driver");
    if (deployment_.find("ACT") != std::string::npos ||
        deployment_.find("DRI") != std::string::npos ||
        deployment_.find("ALL") != std::string::npos) {
      thread_pool_->enqueue([this] { this->actor_control_->run(); });
    }
    if (deployment_.find("CHA") != std::string::npos ||
        deployment_.find("DRI") != std::string::npos ||
        deployment_.find("ALL") != std::string::npos) {
      thread_pool_->enqueue([this] { this->chassis_driver_->run(); });
    }
    if (deployment_.find("GPS") != std::string::npos ||
        deployment_.find("DRI") != std::string::npos ||
        deployment_.find("ALL") != std::string::npos) {
      thread_pool_->enqueue([this] { this->gps_driver_->run(); });
    }
    if (deployment_.find("IMU") != std::string::npos ||
        deployment_.find("DRI") != std::string::npos ||
        deployment_.find("ALL") != std::string::npos) {
      thread_pool_->enqueue([this] { this->imu_driver_->run(); });
    }
  }
  void create_perc_workflow(pattern::Workflow* wf) {
    // sensor
    lane_line_sensor_ = std::make_shared<LaneLineSensor>(nh_, "lane line info", "LAN:PERC:ALL", "bridge", "lane info");
    object_sensor_ = std::make_shared<ObjectSensor>(nh_, "front vision object", "OBJ:PERC:ALL", "bridge", "front vision");
    traffic_light_sensor_ = std::make_shared<TrafficLightSensor>(nh_, "front vision lights", "TRL:PERC:ALL", "bridge", "front vision");
    park_freespace_sensor_ = std::make_shared<ParkFreespaceSensor>(nh_, "surround vision parkfreespace", "PAS:PERC:ALL", "bridge", "surround vision");
    // vision detector
    lane_line_ = std::make_shared<LaneLine>(nh_, "lane line detector", "LINE:PERC:ALL", "PERC", "lane_line");
    front_vision_detector_ = std::make_shared<FrontVisionDetector>(nh_, "front vision detector", "VIS:PERC:ALL", "PERC", "front vision");
    surround_vision_detector_ = std::make_shared<SurroundVisionDetector>(nh_, "surround vision detector", "SUR:PERC:ALL", "PERC", "front vision");
//    if (deployment_.find("LAN") != std::string::npos ||
//        deployment_.find("PERC") != std::string::npos ||
//        deployment_.find("ALL") != std::string::npos) {
//      thread_pool_->enqueue([this] { this->lane_line_sensor_->run(); });
//    }
    if (deployment_.find("OBJ") != std::string::npos ||
        deployment_.find("PERC") != std::string::npos ||
        deployment_.find("ALL") != std::string::npos) {
      thread_pool_->enqueue([this] { this->object_sensor_->run(); });
    }
    if (deployment_.find("TRL") != std::string::npos ||
        deployment_.find("PERC") != std::string::npos ||
        deployment_.find("ALL") != std::string::npos) {
      thread_pool_->enqueue([this] { this->traffic_light_sensor_->run(); });
    }
    if (deployment_.find("PAS") != std::string::npos ||
        deployment_.find("PERC") != std::string::npos ||
        deployment_.find("ALL") != std::string::npos) {
      thread_pool_->enqueue([this] { this->park_freespace_sensor_->run(); });
    }
//    if (deployment_.find("LINE") != std::string::npos ||
//        deployment_.find("PERC") != std::string::npos ||
//        deployment_.find("ALL") != std::string::npos) {
//      thread_pool_->enqueue([this] { this->lane_line_->run(); });
//    }
    if (deployment_.find("VIS") != std::string::npos ||
        deployment_.find("PERC") != std::string::npos ||
        deployment_.find("ALL") != std::string::npos) {
      thread_pool_->enqueue([this] { this->front_vision_detector_->run(); });
    }
    if (deployment_.find("SUR") != std::string::npos ||
        deployment_.find("PERC") != std::string::npos ||
        deployment_.find("ALL") != std::string::npos) {
      thread_pool_->enqueue([this] { this->surround_vision_detector_->run(); });
    }
  }
 private:
  stoic::cm::NodeHandle &nh_;
  std::string deployment_;
  std::unique_ptr<ThreadPool> thread_pool_;
  std::shared_ptr<ActorControl> actor_control_;
  std::shared_ptr<ChassisDriver> chassis_driver_;
  std::shared_ptr<GpsDriver> gps_driver_;
  std::shared_ptr<ImuDriver> imu_driver_;
  std::shared_ptr<LaneLineSensor> lane_line_sensor_;
  std::shared_ptr<ObjectSensor> object_sensor_;
  std::shared_ptr<TrafficLightSensor> traffic_light_sensor_;
  std::shared_ptr<ParkFreespaceSensor> park_freespace_sensor_;
  std::shared_ptr<CanbusCore> sim_canbus_core_;
  std::shared_ptr<MsfLocalizationCore> sim_msf_localization_core_;
  std::shared_ptr<LaneLine> lane_line_;
  std::shared_ptr<FrontVisionDetector> front_vision_detector_;
  std::shared_ptr<SurroundVisionDetector> surround_vision_detector_;
};
} // namespace stoic::app
