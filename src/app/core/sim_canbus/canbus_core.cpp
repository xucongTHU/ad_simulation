
//
// Created by xucong on 23-8-7.
//
#include "app/core/sim_canbus/canbus_core.h"
#include "common/log/Logger.h"
#include "common/config.h"

using namespace stoic;
using namespace stoic::app::core;
static const char* LOG_TAG =  "canbus";

constexpr double kMaxPercentage = 100.0;
constexpr double kMaxSteerangle = 8.726646;
namespace stoic::app::core {
void CanbusCore::ChassisDriverCallback(const std::shared_ptr<sim_chassis::Chassis>& msg) {
  last_sim_chassis_timestamp_ = Timer::now_sec();
  latest_sim_chassis = *msg;
  is_sim_chassis_ready = true;
  LOG_INFO("canbus instance, received topic is: %s, , Msg Type is : %s",
           FLAGS_sim_chassis_topic.c_str(),
           reflection<::sim_chassis::Chassis>::fullName().c_str());
  LOG_INFO("[Canbus] 接收的序列号: %u,时间戳: %ld",msg->header.seq, msg->header.stamp);
}

void CanbusCore::run() {
  static stoic::cm::Subscriber sub_canbus = nh_ptr_->subscribe<::sim_chassis::Chassis, true>(
      FLAGS_sim_chassis_topic, 1,
      [this](auto && canbus_data) { ChassisDriverCallback(std::forward<decltype(canbus_data)>(canbus_data)); });
  using ChassisType = ::caic_sensor::Canbus;
  vehicle_param_.CopyFrom(VehicleConfigHelper::Instance()->GetConfig().vehicle_param());
  static stoic::cm::Publisher pub_chassis =
      nh_ptr_->advertise<ChassisType, true>(FLAGS_chassis_topic, 1);
  Rate rate(100);
  while (stoic::cm::ok()) {
    ChassisType chassis_result;
    PublishSimChassis(&chassis_result);
    if (is_sim_chassis_ready)
      pub_chassis.publish<ChassisType, true>(chassis_result);
    rate.sleep();
  }
}
bool CanbusCore::PublishSimChassis(caic_sensor::Canbus *chassis_) {
  if (!is_sim_chassis_ready) {
    LOG_ERROR("Sim chassis msg is not ready!");
    return false;
  }
  chassis_->header.stamp = Timer::now_us();
  chassis_->header.seq = seq_num++;
  std::string module_name = "sim_chassis";
  strncpy(chassis_->header.module_name, module_name.data(), STD_MODULE_NAME_SIZE);
#if defined MW_ROS_IPC || defined MW_ROS_ORIN
  chassis_->driving_mode = caic_sensor::DrivingMode::COMPLETE_AUTO_MODE;
  chassis_->error_status = caic_sensor::ErrorState::NO_ERROR;
  chassis_->chassis_info.esp.vehicle_speed = latest_sim_chassis.chassis_motion.vehicle_speed;
  chassis_->chassis_info.gear_info = caic_std::GearState::DRIVE;
  static float steering_angle_percentage = 0.0;
  steering_angle_percentage = static_cast<float>(latest_sim_chassis.steering.steering_wheel_info.angle /
      vehicle_param_.max_steer_angle() * 100);
  chassis_->chassis_info.eps.steering_wheel_info.angle = latest_sim_chassis.steering.steering_wheel_info.angle;
  chassis_->chassis_info.yrs.acceleration_x = latest_sim_chassis.chassis_motion.acceleration_x;
  chassis_->chassis_info.esp.brake_pedal_state = false;
#elif defined MW_ART_IPC || defined MW_ART_ORIN
  chassis_->available =
      caic_sensor::Canbus::CHASSIS_INFO |
      caic_sensor::Canbus::AUTO_CONTROL_STATE |
      caic_sensor::Canbus::DRIVING_MODE;
  chassis_->driving_mode = caic_sensor::DrivingMode::COMPLETE_AUTO_MODE;
  chassis_->error_status = caic_sensor::ErrorState::NO_ERROR;
  chassis_->door_switch_status = 0;
  chassis_->chassis_info.gear_info = caic_std::GearState::DRIVE;
  chassis_->chassis_info.yrs.acceleration_x = latest_sim_chassis.chassis_motion.acceleration_x;
  chassis_->chassis_info.esp.vehicle_speed = latest_sim_chassis.chassis_motion.vehicle_speed;
  chassis_->chassis_info.esp.brake_pedal_state = false;
  chassis_->body_info.light_state.position_light_state = true;
//  chassis_->chassis_info.eps.steering_wheel_info.angle =
//      latest_sim_chassis.steering.steering_wheel_info.angle / kMaxSteerangle * kMaxPercentage; // steering percentage
  chassis_->chassis_info.eps.hand_steering_torque = latest_sim_chassis.steering.hand_steering_torque;
  LOG_ERROR("[Canbus] steering torque : %.3f, speed : %.3f, acceleration : %.3f",
            chassis_->chassis_info.eps.hand_steering_torque,
            chassis_->chassis_info.esp.vehicle_speed,
            chassis_->chassis_info.yrs.acceleration_x);
  chassis_->chassis_info.esp.driving_direction = caic_sensor::DrivingDirection::FORWARD;
  if ((sim_std::GearState)latest_sim_chassis.powertrain.gear_state == sim_std::GearState::REVERSE) {
    chassis_->chassis_info.esp.driving_direction = caic_sensor::DrivingDirection::BACKWARD;
    LOG_INFO("[Canbus] 处于倒车模式, 档位：%d", chassis_->chassis_info.gear_info);
  }
  chassis_->chassis_info.esp.wheel_speed_info.fl = latest_sim_chassis.wheels.wheel_speed_info.fl;
  chassis_->chassis_info.esp.wheel_speed_info.fr = latest_sim_chassis.wheels.wheel_speed_info.fr;
  chassis_->chassis_info.esp.wheel_speed_info.rl = latest_sim_chassis.wheels.wheel_speed_info.rl;
  chassis_->chassis_info.esp.wheel_speed_info.rr = latest_sim_chassis.wheels.wheel_speed_info.rr;
  chassis_->chassis_info.epb.epb_working_state = caic_sensor::EPBWorkingState::UNKNOW;
  // turn light
  chassis_->body_info.light_state.turning_state_left = false;
  chassis_->body_info.light_state.turning_state_right = false;
#endif
  return true;
}
} // namespace stoic::app::core
