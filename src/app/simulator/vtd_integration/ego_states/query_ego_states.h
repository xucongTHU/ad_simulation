//
// Created by xucong on 23-8-3.
//

#ifndef SIMULATOR_VTD_INTEGRATION_QUERY_EGO_STATES_H_
#define SIMULATOR_VTD_INTEGRATION_QUERY_EGO_STATES_H_

#include "yaml-cpp/yaml.h"
#include "memory"
#include "sim_interface.h"

#include "common/time/timer.h"
#include "common/network/udpsocket_server.h"
#include "common/network/udpsocket_client.h"

#include "simulator/vtd_integration/vtd_dependencies/yaml_util.h"
#include "simulator/vtd_integration/ego_states/ego_states_base.h"


namespace simulator::vtd_integration {

class QueryVehicleStates : public stoic::UdpSocketServer<sim_chassis::EgoStates>,
                           public stoic::UdpSocketClient<RDB_EGO_STATE_t> {
 public:
  QueryVehicleStates();

  ~QueryVehicleStates() {
    delete egoData;
  }
  void* msgHandle(const sim_chassis::EgoStates & msg);
  void run();

  bool sDataReceived = false;

 private:
  RDB_EGO_STATE_t* egoData;
  YAML::Node node;
  std::string remote_ip;
  int bind_port;


};

} // namespace simulator::vtd_integration
#endif //SIMULATOR_VTD_INTEGRATION_QUERY_EGO_STATES_H_
