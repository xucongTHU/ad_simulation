#ifndef SIM_INTERFACE_SIM_CHASSIS_INTERFACE_H
#define SIM_INTERFACE_SIM_CHASSIS_INTERFACE_H

#include <stdint.h>

#include <string>
#include <vector>

#include "sim_interface/sim_std.h"

namespace sim_chassis {

struct ChassisMeta {
  int64_t start_timestamp_us;
  int64_t end_timestamp_us;
  int64_t sensor_timestamp_us;
};

/**
 * @brief DrivingDirection
 *  indicates driving direction of the vehicle
 */
enum class DrivingDirection : uint8_t {
  STOP = 0,      ///< STOP
  FORWARD,       ///< FORWARD
  BACKWARD,      ///< BACKWARD
  INVALID_VALUE, ///< INVALID VALUE
};

/// indicates the steering wheel info
struct SteeringWheelInfo {
  uint64_t avaliable;
  double angle;    ///< THE ANGLE OF STEERING WHEEL
  bool angle_sign; ///< THE ANGLE SIGN OF STEERING WHEEL
  double speed;    ///< THE SPEED OF STEERING WHEEL
  bool speed_sign; ///< THE SPEED SIGN OF STEERING WHEEL
  double rl;       ///< THE WHEEL STEER ANGLE OF REAR LEFT
  double rr;       ///< THE WHEEL STEER ANGLE OF REAR RIGHT
  double fl;       ///< THE WHEEL STEER ANGLE OF FRONT LEFT
  double fr;       ///< THE WHEEL STEER ANGLE OF FRONT RIGHT
  enum : uint64_t {
    ANGLE = 1 << 0,
    ANGLE_SIGN = 1 << 1,
    SPPED = 1 << 2,
    SPEED_SIGN = 1 << 3,
    RL = 1 << 4,
    RR = 1 << 5,
    FL = 1 << 6,
    FR = 1 << 7,
  };
};

struct WheelSpeedInfo {
  uint64_t avaliable;
  double rl; ///< THE WHEEL SPEED OF REAR LEFT
  double rr; ///< THE WHEEL SPEED OF REAR RIGHT
  double fl; ///< THE WHEEL SPEED OF FRONT LEFT
  double fr; ///< THE WHEEL SPEED OF FRONT RIGHT
  enum : uint64_t {
    RL = 1 << 0,
    RR = 1 << 1,
    FL = 1 << 2,
    FR = 1 << 3,
  };
};

struct WheelRotationInfo {
  uint64_t avaliable;
  double rl; ///< THE WHEEL ROTATION(EULER) OF REAR LEFT
  double rr; ///< THE WHEEL ROTATION(EULER) OF REAR RIGHT
  double fl; ///< THE WHEEL ROTATION(EULER) OF FRONT LEFT
  double fr; ///< THE WHEEL ROTATION(EULER) OF FRONT RIGHT
  enum : uint64_t {
    RL = 1 << 0,
    RR = 1 << 1,
    FL = 1 << 2,
    FR = 1 << 3,
  };
};

struct WheelSpinInfo {
  uint64_t avaliable;
  double rl; ///< THE WHEEL DRIVELINE SPEED OF REAR LEFT
  double rr; ///< THE WHEEL DRIVELINE SPEED OF REAR RIGHT
  double fl; ///< THE WHEEL DRIVELINE SPEED OF FRONT LEFT
  double fr; ///< THE WHEEL DRIVELINE SPEED OF FRONT RIGHT
  enum : uint64_t {
    RL = 1 << 0,
    RR = 1 << 1,
    FL = 1 << 2,
    FR = 1 << 3,
  };
};

struct EffectiveRadiusInfo {
  uint64_t avaliable;
  double rl; ///< THE TIRE EFFECTIVE ROLLING RAD OF REAR LEFT
  double rr; ///< THE TIRE EFFECTIVE ROLLING RAD OF REAR RIGHT
  double fl; ///< THE TIRE EFFECTIVE ROLLING RAD OF FRONT LEFT
  double fr; ///< THE TIRE EFFECTIVE ROLLING RAD OF FRONT RIGHT
  enum : uint64_t {
    RL = 1 << 0,
    RR = 1 << 1,
    FL = 1 << 2,
    FR = 1 << 3,
  };
};

/// indicates lateral signal
struct Steering {
  uint64_t avaliable;
  SteeringWheelInfo steering_wheel_info; ///< INDICATES THE STEERING WHEEL INFO
  double hand_steering_torque;           ///< HAND STEERING TORQUE
  enum : uint64_t {
    STEERING_WHEEL_INFO = 1 << 0,
    HAND_STEERING_TORQUE = 1 << 1,
  };
};

/// indicate lon signal
struct ChassisMotion {
  uint64_t avaliable;
  DrivingDirection driving_direction; ///< INDICATES DRIVING DIRECTION OF THE VEHICLE
  double master_cylinder_pressure;         ///< MASTER CYLINDER PRESSURE
  double vehicle_speed;                    ///< VEHICLE SPEED
  bool vehicle_speed_q;                    ///< VEHICLE SPPED Q
  double acceleration_x;                   ///< THE ACCELERATION OF X
  double acceleration_y;                   ///< THE ACCELERATION OF Y
  double acceleration_pedal_position;      ///< ACCELERACTION PEDAL POSITION
  double calc_acceleration_pedal_position; ///< THE CALCULATION OF ACCELERATION
  bool brake_pedal_state;                  ///< THE STATUS OF BRAKE PEADA
  enum : uint64_t {
    DRIVING_DIRECTION = 1 << 0,
    MASTER_CYLINDER_PRESSURE = 1 << 1,
    VEHICLE_SPEED = 1 << 2,
    VEHICLE_SPEED_Q = 1 << 3,
    ACCELERATION_X = 1 << 4,
    ACCELERATION_Y = 1 << 5,
    ACCELERATION_PEDAL_POSITION = 1 << 6,
    CALC_ACCELERATION_PEDAL_POSITION = 1 << 7,
    BRAKE_PEDAL_STATE = 1 << 8,
  };
};

struct Powertrain {
  sim_std::GearState gear_state;
  enum : uint64_t {
    GEAR_STATE = 1 << 0
  };
};

struct Wheels {
  WheelSpeedInfo wheel_speed_info;         ///< INDICATES THE WHEEL SPPED
  WheelRotationInfo wheel_rotation_info;   ///< INDICATES THE WHEEL ROTATION
  WheelSpinInfo wheel_spin_info;           ///< INDICATES THE WHEEL DRIVELINE SPEED
  enum : uint64_t {
    WHEEL_SPEED_INFO = 1 << 0
  };
};

struct Tires {
  EffectiveRadiusInfo effective_radius_info;
  enum : uint64_t {
    EFFECTIVE_RADIUS_INFO = 1 << 0
  };
};

 struct Chassis : public sim_std::MessageBase {
  sim_std::HeaderPOD header;
  uint64_t avaliable;
  Steering steering;
  ChassisMotion chassis_motion;
  Powertrain powertrain;
  Wheels wheels;
  Tires tires;
};

struct ChassisUdp {
  double simTime;
  uint32_t simFrame;
  uint64_t available;
  Steering steering;
  ChassisMotion chassis_motion;
  uint8_t gear_state;
  Wheels wheels;
  Tires tires;
};

struct EgoStates{
  double simTime;
  uint32_t simFrame;
  uint64_t available;
  sim_localization::LocalizationPose pose;
  Steering steer_info;
  Wheels wheel_info;
  Tires tire_info;
};

} // namespace sim_chassis

#endif



