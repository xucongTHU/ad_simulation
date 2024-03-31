#ifndef ad_INTERFACE_ad_CONTROL_INTERFACE_H
#define ad_INTERFACE_ad_CONTROL_INTERFACE_H

#include <stdint.h>

#include "ad_interface/ad_std.h"

namespace ad_control {

/**
 * time cost
 */
struct ControlMeta {
  int64_t planning_timestamp_us;  // input timestamp of planing
  int64_t localization_timestamp_us;  // input timestamp of localization
  int64_t chassis_timestamp_us; // input timestamp of chassis
  int64_t start_timestamp_us; // timestamp of executing control algorithm
  int64_t finish_timestamp_us;  // timestamp of finishing control algorithm
};

/**
 * driving mode
 */
enum class DrivingMode : uint8_t {
  COMPLETE_MANUAL = 0,  // request for takeovering
  COMPLETE_AUTO_DRIVE,  // request for auto driving
  AUTO_STEER_ONLY,  // request for lateral control
  AUTO_SPEED_ONLY,  // request for longitudial control
  EMERGENCY_MODE, // request for takeovering
};

/**
 * acc request type
 */
enum class AccReqSt : uint8_t {
  NO_REQUEST = 0, // no request
  ACC_REQUEST,   // request for accelerating
  DEC_REQUEST,  // request for decelerating
};

/**
 * epb request type
 */
enum class EPBReqSt : uint8_t {
  NO_REQUEST = 0, // no request
  LOCK_REQUEST, // request for locking parking brake
  RELEASE_REQUEST,  // request for releasing parking brake
};

/**
 * engine on or off request type
 */
enum class EngineReqSt : uint8_t {
  NO_REQUEST = 0, // no request
  ENGINE_ON_REQUEST,   // request for starting engine
  ENGINE_OFF_REQUEST, // request for engine off
};

/**
 * warning request type
 */
struct WarningReq {
  uint8_t fcw_req;  // forward collision warning request
  uint8_t aeb_req;  // auto emgency brake warning request
  uint8_t bsd_req;  // blind spot detection warning request
  uint8_t ldw_req;  // lane departure warning request
  uint8_t dow_req;  // door open warning request
  uint8_t rcw_req;  // rear collision warn request
  uint8_t fcta_req; // front crossing traffic alert request
  uint8_t rcta_req; // rear crossing traffic alert request
  uint8_t ciw_req;  // cut in warning request
  uint8_t isa_req;  // intelligent speed warning request
};

/**
 * function request type
 */
struct FunctionReq {
  bool prefill_req;    // prefill brake request for aeb
  bool jerk_brake_req; // jerk brake request for aeb
  uint8_t aeb_st;      // aeb deceleration request
  bool tja_req;        // traffic jam assist request
  bool ja_req;         // junction assist request
  bool acc_req;        // adaptitive cruise control request
  bool lka_req;        // lane keeping assist request
  bool lca_req;        // lane centering assist request
  bool islc_req;       // intelligent speed limit control request
  bool ciw_req;        // cut it warning/brake request
  bool fcta_b_req;     // front crossing traffic alert and brake request
  bool rcta_b_req;     // rear crossing traffic alert and brake request
};

/**
 * control command for canbus
 */
struct ControlCommand : public ad_std::MessageBase {
  ad_std::HeaderPOD header;
  ControlMeta meta;
  uint64_t available;    // binary,1 for available,0 for unavailable
  float throttle;        // throttle opening in percentage
  float brake;           // brake master cylinder pressure unit<mpa>
  float steering_target; // steering wheel angle in percentage
  float steering_rate;   // steering wheel angle rate rad/s
  float steering_torque; // steering wheel torque nm
  float speed;           // target speed m/s
  float acceleration;    // target acceleration m/s^2
  AccReqSt acc_req_st;  // request type for acc 
  float parking_distance; // parking distance in meter m
  EPBReqSt parking_brake; // request type for epb
  WarningReq warn_req;  // request type for warning
  FunctionReq fun_req;  // request type for function 
  EngineReqSt engine_on_off; //enable starting or power off engine or electric motor
  bool is_estop;             // enable emergency stopping brake
  DrivingMode driving_mode;  // driving mode
  ad_std::GearState gear_state;         // gear position
  ad_std::VehicleSignal vehicle_signal; // vehicle signal
  bool stand_still;                       // enable stand still
  bool rearview_mirror;                   // enable rearview mirror fold

  enum : uint64_t {
    CONTROL_COMMAND_THROTTLE = 1 << 0,
    CONTROL_COMMAND_BRAKE = 1 << 1,
    CONTROL_COMMAND_STEERING_TARGET = 1 << 2,
    CONTROL_COMMAND_STEERING_RATE = 1 << 3,
    CONTROL_COMMAND_STEERING_TORQUE = 1 << 4,
    CONTROL_COMMAND_SPEED = 1 << 5,
    CONTROL_COMMAND_ACCELERATION = 1 << 6,
    CONTROL_COMMAND_ACC_REQ_ST = 1 << 7,
    CONTROL_COMMAND_PARKING_DISTANCE = 1 << 8,
    CONTROL_COMMAND_PARKING_BRAKE = 1 << 9,
    CONTROL_COMMAND_WARN_REQ = 1 << 10,
    CONTROL_COMMAND_FUN_REQ = 1 << 11,
    CONTROL_COMMAND_ENGINE_ON_OFF = 1 << 12,
    CONTROL_COMMAND_IS_ESTOP = 1 << 13,
    CONTROL_COMMAND_DRIVING_MODE = 1 << 14,
    CONTROL_COMMAND_GEAR_STATE = 1 << 15,
    CONTROL_COMMAND_VEHICLE_SIGNAL = 1 << 16,
    CONTROL_COMMAND_STAND_STILL = 1 << 17,
    CONTROL_COMMAND_REARVIEW_MIRROR = 1 << 18,
  };
};

} // namespace ad_control

#endif
