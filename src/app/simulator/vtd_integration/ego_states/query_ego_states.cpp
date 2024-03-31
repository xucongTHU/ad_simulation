//
// Created by xucong on 23-8-3.
//

#include "app/simulator/vtd_integration/ego_states/query_ego_states.h"

#include <iostream>
#include <cstring>

namespace simulator::vtd_integration {

QueryVehicleStates::QueryVehicleStates() {
  node = YAML::LoadFile("/home/autopilot/stoic/src/app/configs/vtd/config.yaml");
  std::string key = "ego_states";
  YAML::Node result = YamlUtil::GetValue(node, key);
  if (!result.IsNull() && result.IsMap()) {
    bind_port = result["bind_port"].as<int>();
  } else {
    std::cout << "Key '" << key << "' not found in the YAML data." << std::endl;
  }
  connectToUdpClient(bind_port);
  connectToUdpServer("127.0.0.1", RDB_FEEDBACK_PORT);
}

void* QueryVehicleStates::msgHandle(const sim_chassis::EgoStates &msg) {
  egoData = new RDB_EGO_STATE_t;
  memset(egoData, 0, sizeof(RDB_EGO_STATE_t));

  // Header
  egoData->hdr.magicNo = RDB_MAGIC_NO;
  egoData->hdr.version = RDB_VERSION;
  egoData->hdr.headerSize = sizeof(RDB_MSG_HDR_t);
  egoData->hdr.dataSize = 5 * sizeof(RDB_MSG_ENTRY_HDR_t) + sizeof(RDB_OBJECT_STATE_t) + 4 * sizeof(RDB_WHEEL_t) + sizeof(RDB_TRIGGER_t);
  egoData->hdr.frameNo = msg.simFrame;
  egoData->hdr.simTime = msg.simTime;

  // StartOfFrame
  egoData->entrySOF.headerSize = sizeof(RDB_MSG_ENTRY_HDR_t);
  egoData->entrySOF.dataSize = 0;
  egoData->entrySOF.elementSize = 0;
  egoData->entrySOF.pkgId = RDB_PKG_ID_START_OF_FRAME;
  egoData->entrySOF.flags = 0;

  // trigger
  egoData->entryTrigger.headerSize = sizeof(RDB_MSG_ENTRY_HDR_t);
  egoData->entryTrigger.dataSize   = sizeof(RDB_TRIGGER_t);
  egoData->entryTrigger.elementSize = sizeof(RDB_TRIGGER_t);
  egoData->entryTrigger.pkgId = RDB_PKG_ID_TRIGGER;
  egoData->entryTrigger.flags = 0;
  egoData->rdbTrigger.deltaT  = 0.02;
  egoData->rdbTrigger.frameNo = 1;

  // Ego states
  egoData->entryEgo.headerSize = sizeof(RDB_MSG_ENTRY_HDR_t);
  egoData->entryEgo.dataSize = sizeof(RDB_OBJECT_STATE_t);
  egoData->entryEgo.elementSize = sizeof(RDB_OBJECT_STATE_t);
  egoData->entryEgo.pkgId = RDB_PKG_ID_OBJECT_STATE;
  egoData->entryEgo.flags = RDB_PKG_FLAG_EXTENDED;

  sprintf(egoData->egoState.base.name, "&s", "Ego");
  egoData->egoState.base.id = 1;
  egoData->egoState.base.category = RDB_OBJECT_CATEGORY_PLAYER;
  egoData->egoState.base.type = RDB_OBJECT_TYPE_PLAYER_CAR;
  egoData->egoState.base.visMask = RDB_OBJECT_VIS_FLAG_ALL;
  egoData->egoState.base.geo.dimX = 5.209;
  egoData->egoState.base.geo.dimY = 2.20;
  egoData->egoState.base.geo.dimZ = 1.731;
  egoData->egoState.base.pos.x = msg.pose.pose_info.x;
  egoData->egoState.base.pos.y = msg.pose.pose_info.y;
  egoData->egoState.base.pos.z = msg.pose.pose_info.z;
  egoData->egoState.base.pos.h = msg.pose.angle.roll;
  egoData->egoState.base.pos.p = msg.pose.angle.pitch;
  egoData->egoState.base.pos.r = msg.pose.angle.roll;
  egoData->egoState.base.pos.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;
  egoData->egoState.base.pos.type = RDB_COORD_TYPE_INERTIAL;
  egoData->egoState.base.pos.system = 0;
  egoData->egoState.ext.speed.x = msg.pose.linear_velocity_info.x;
  egoData->egoState.ext.speed.y = msg.pose.linear_velocity_info.y;
  egoData->egoState.ext.speed.z = msg.pose.linear_velocity_info.z;
  egoData->egoState.ext.speed.h = msg.pose.angular_velocity_vrf_info.z;
  egoData->egoState.ext.speed.p = msg.pose.angular_velocity_vrf_info.y;
  egoData->egoState.ext.speed.r = msg.pose.angular_velocity_vrf_info.x;
  egoData->egoState.ext.accel.x = msg.pose.linear_acceleration_vrf_info.x;
  egoData->egoState.ext.accel.y = msg.pose.linear_acceleration_vrf_info.y;
  egoData->egoState.ext.accel.z = msg.pose.linear_acceleration_vrf_info.z;
  egoData->egoState.ext.accel.h = msg.pose.angular_acceleration_vrf_info.x;
  egoData->egoState.ext.accel.p = msg.pose.angular_acceleration_vrf_info.y;
  egoData->egoState.ext.accel.r = msg.pose.angular_acceleration_vrf_info.z;
  egoData->egoState.ext.speed.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;
  egoData->egoState.ext.speed.type = RDB_COORD_TYPE_INERTIAL;
  egoData->egoState.ext.speed.system = 0;
  egoData->egoState.ext.accel.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;
  egoData->egoState.ext.accel.type = RDB_COORD_TYPE_INERTIAL;
  egoData->egoState.ext.accel.system = 0;

  // Wheel state
  egoData->entryWheel.headerSize = sizeof(RDB_MSG_ENTRY_HDR_t);
  egoData->entryWheel.dataSize = 4 * sizeof(RDB_WHEEL_t);
  egoData->entryWheel.elementSize = sizeof(RDB_WHEEL_t);   // FOUR elements
  egoData->entryWheel.pkgId = RDB_PKG_ID_WHEEL;
  egoData->entryWheel.flags = RDB_PKG_FLAG_EXTENDED;
  egoData->wheels[0].base.playerId = 1;
  egoData->wheels[0].base.id = 0;
  egoData->wheels[0].base.radiusStatic = msg.tire_info.effective_radius_info.fl;
  egoData->wheels[0].base.springCompression = 0;
  egoData->wheels[0].base.rotAngle = msg.wheel_info.wheel_rotation_info.fl;
  egoData->wheels[0].base.slip = 0.5;
  egoData->wheels[0].base.steeringAngle = msg.steer_info.steering_wheel_info.fl;
  egoData->wheels[0].ext.vAngular = msg.wheel_info.wheel_spin_info.fl;

  egoData->wheels[1].base.playerId = 1;
  egoData->wheels[1].base.id = 1;
  egoData->wheels[1].base.radiusStatic = msg.tire_info.effective_radius_info.fr;
  egoData->wheels[1].base.springCompression = 0;
  egoData->wheels[1].base.rotAngle = msg.wheel_info.wheel_rotation_info.fr;
  egoData->wheels[1].base.slip = 0.5;
  egoData->wheels[1].base.steeringAngle = msg.steer_info.steering_wheel_info.fr;
  egoData->wheels[1].ext.vAngular = msg.wheel_info.wheel_spin_info.fr;

  egoData->wheels[2].base.playerId = 1;
  egoData->wheels[2].base.id = 2;
  egoData->wheels[2].base.radiusStatic = msg.tire_info.effective_radius_info.rl;
  egoData->wheels[2].base.springCompression = 0;
  egoData->wheels[2].base.rotAngle = msg.wheel_info.wheel_rotation_info.rl;
  egoData->wheels[2].base.slip = 0.5;
  egoData->wheels[2].base.steeringAngle = msg.steer_info.steering_wheel_info.rl;
  egoData->wheels[2].ext.vAngular = msg.wheel_info.wheel_spin_info.rl;

  egoData->wheels[3].base.playerId = 1;
  egoData->wheels[3].base.id = 3;
  egoData->wheels[3].base.radiusStatic = msg.tire_info.effective_radius_info.rr;
  egoData->wheels[3].base.springCompression = 0;
  egoData->wheels[3].base.rotAngle = msg.wheel_info.wheel_rotation_info.rr;
  egoData->wheels[3].base.slip = 0.5;
  egoData->wheels[3].base.steeringAngle = msg.steer_info.steering_wheel_info.rr;
  egoData->wheels[3].ext.vAngular = msg.wheel_info.wheel_spin_info.rr;

  //EndOfFrame
  egoData->entryEOF.headerSize = sizeof(RDB_MSG_ENTRY_HDR_t);
  egoData->entryEOF.dataSize = 0;
  egoData->entryEOF.elementSize = 0;
  egoData->entryEOF.pkgId = RDB_PKG_ID_END_OF_FRAME;
  egoData->entryEOF.flags = 0;

  // trigger flags
  sDataReceived = true;

  fprintf( stderr, "send ego state to vtd\n" );
  fprintf( stderr, "    simTime = %.3lf, simFrame = %u\n", egoData->hdr.simTime, egoData->hdr.frameNo );

  return static_cast<void*>(egoData);

// useage
//  void* voidPtr = queryVehicleStates(msg);
//  sensor_data::MSG_EGO_STATES_t* msg_ego_states = static_cast<sensor_data::MSG_EGO_STATES_t*>(voidPtr);

}

void QueryVehicleStates::run() {
  // recv ego vehicle states
  sim_chassis::EgoStates latest_ego;
  recvData(latest_ego);
  fprintf( stderr, "recv ego vehicle state\n" );
  fprintf( stderr, "    simTime = %.3lf, simFrame = %u\n", latest_ego.simTime, latest_ego.simFrame );
  fprintf( stderr, "    pos x/y/z = %.3f/%.3f/%.3f\n", latest_ego.pose.pose_info.x, latest_ego.pose.pose_info.y, latest_ego.pose.pose_info.z);

  //
  void* voidPtr = msgHandle(latest_ego);
  auto msg_ego = static_cast<RDB_EGO_STATE_t *>(voidPtr);
  sendData(*msg_ego);
}

} // namespace simulator::vtd_integration

int main(int argc, char** argv) {
  simulator::vtd_integration::QueryVehicleStates query_vehicle_states;
  while (true) {
    query_vehicle_states.run();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

  }
}
