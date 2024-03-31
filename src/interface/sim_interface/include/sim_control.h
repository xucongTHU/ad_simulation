#ifndef SIM_INTERFACE_SIM_CONTROL_INTERFACE_H
#define SIM_INTERFACE_SIM_CONTROL_INTERFACE_H

#include <stdint.h>

#include <string>
#include <vector>

#include "sim_interface/sim_std.h"

namespace sim_control {

struct ControlMeta {
  int64_t start_timestamp_us;
  int64_t end_timestamp_us;
  int64_t sensor_timestamp_us;
};

struct ControlCommand : public sim_std::MessageBase {
  sim_std::HeaderPOD header;
  ControlMeta mata;
  double steering_angle;    /**<steering wheel angle rad*/
  double acceleration;      /**<target acceleration m/s^2*/
};

struct ControlUdp {
  double simTime;
  uint32_t simFrame;
  uint64_t available;
  double steering_angle;    /**<steering wheel angle rad*/
  double acceleration;      /**<target acceleration m/s^2*/
};

} // namespace sim_control

#endif
