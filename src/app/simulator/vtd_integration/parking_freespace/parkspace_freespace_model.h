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

using namespace sim_ground_truth;
namespace stoic::simulator {
class ParkFreespaceModel : public pattern::Task<ParkFreespaceModel>,
                                public simulation::UdpSocketClient<ParkingSpaces>{
 public:
  template <typename... OptionArgs>
  ParkFreespaceModel(OptionArgs&&... option_args) : Task(std::forward<OptionArgs>(option_args)...) {}
  ParkFreespaceModel(const pattern::TaskOptions& task_options) : Task(task_options) {}
  ~ParkFreespaceModel() {}
  void run() override;
  void handleParkingSpace(const double &simTime, const unsigned int &simFrame, RDB_OBJECT_STATE_t &item, bool isExtended, bool isSensor, int parking_space_size);
 private:
  void init();
  int openPort(int & descriptor, int portNo, const char* serverAddr);
  void initConnections();
  void initConnection(Connection_t &conn);
  void readConnection(Connection_t &conn, bool waitForMessage, bool verbose);
  void parseRDBMessage( Connection_t & conn, RDB_MSG_t* msg );
  void parseRDBMessageEntry( Connection_t & conn, const double & simTime, const unsigned int & simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr );
  void sendParkingspace(const double &simTime, const unsigned int &simFrame, const int &parking_space_size);
 private:
  RDB_OBJECT_STATE_t mSurVisionState;
  RDB_OBJECT_STATE_t mParkspaceState;
  std::shared_ptr<ParkingSpaces> parking_spaces_ = std::make_shared<ParkingSpaces>();
  // connection instances
  Connection_t sConnection[MAX_CONNECTIONS];
  std::string sensor_ip;
  int sensor_port;
  int parkspace_port;

};

} // namespace stoic::simulator
WORKFLOW_ADD_TASK(::stoic::simulator::ParkFreespaceModel)
