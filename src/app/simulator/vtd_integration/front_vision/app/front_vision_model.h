
//
// Created by xucong on 23-12-28.
//
#pragma once
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include "yaml-cpp/yaml.h"
#include "sim_interface.h"
#include "common/network/udpsocket_client.hpp"
#include "simulator/vtd_integration/vtd_dependencies/pattern/task.hpp"
#include "simulator/vtd_integration/vtd_dependencies/common/RDBHandler.hh"
#include "simulator/vtd_integration/vtd_dependencies/common/viRDBIcd.h"
#include "simulator/vtd_integration/vtd_dependencies/udp_port.h"
#include "simulator/vtd_integration/front_vision/lane_line_type/lane_boundary.h"

using namespace sim_ground_truth;
namespace stoic::simulator {
class FrontVisionModel : public pattern::Task<FrontVisionModel>,
                         public simulation::UdpSocketClient<FrontVisionObjects>,
                         public simulation::UdpSocketClient<TrafficLights>,
                         public simulation::UdpSocketClient<LaneLines> {
 public:
  template <typename... OptionArgs>
  FrontVisionModel(OptionArgs&&... option_args) : Task(std::forward<OptionArgs>(option_args)...) {}
  FrontVisionModel(const pattern::TaskOptions& task_options) : Task(task_options) {}
  ~FrontVisionModel() {}
  void run() override;
  void laneHandler(const double &simTime, const unsigned int &simFrame, RDB_ROADMARK_t &item, bool isSensor, int laneSize);
  void roadHandler(const double &simTime, const unsigned int &simFrame, RDB_ROAD_POS_t &item, bool isSensor);
  void objHandler(const double &simTime, const unsigned int &simFrame, RDB_OBJECT_STATE_t &item, bool isExtended, bool isSensor, int objSize);
  void tlightsHandler(const double &simTime, const unsigned int &simFrame, RDB_TRAFFIC_LIGHT_t &item, bool isExtended, bool isSensor, int tlSize);
  void trafficSignHandler(const double &simTime, const unsigned int &simFrame, RDB_TRAFFIC_SIGN_t &item, bool isSensor);
 private:
  void init();
  int openPort(int & descriptor, int portNo, const char* serverAddr);
  void initConnections();
  void initConnection(Connection_t &conn);
  void readConnection(Connection_t &conn, bool waitForMessage, bool verbose);
  void parseRDBMessage( Connection_t & conn, RDB_MSG_t* msg );
  void parseRDBMessageEntry( Connection_t & conn, const double & simTime, const unsigned int & simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr );
  void sendObjectsUdp(const double &simTime, const unsigned int &simFrame, const int &objSize);
  void sendLightsUdp(const double &simTime, const unsigned int &simFrame, const int &lightSize);
  void sendLanesUdp(const double &simTime, const unsigned int &simFrame, const int &laneSize);
 private:
  RDB_ROADMARK_t mLaneLines;
  RDB_ROAD_POS_t mRoadPos;
  RDB_OBJECT_STATE_t mObjState;
  RDB_OBJECT_STATE_t mOwnObject;
  RDB_TRAFFIC_LIGHT_t mLights;
  RDB_TRAFFIC_SIGN_t mSigns;
  std::shared_ptr<FrontVisionObjects> front_vision_objects_ = std::make_shared<FrontVisionObjects>();
  std::shared_ptr<TrafficLights> traffic_lights_ = std::make_shared<TrafficLights>();
  std::shared_ptr<LaneLines> lane_lines_ = std::make_shared<LaneLines>();
  std::shared_ptr<LaneBoundary> lane_boundary_;
  // connection instances
  Connection_t sConnection[MAX_CONNECTIONS];
  std::string sensor_ip;
  int sensor_port;
  int obj_port;
  int light_port;
  int lane_port;

};

} // namespace stoic::simulator
WORKFLOW_ADD_TASK(::stoic::simulator::FrontVisionModel)
