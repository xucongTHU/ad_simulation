
//
// Created by xucong on 23-8-7.
//
#include "app/core/sim_msf_localization/app/msf_loccaization_core.h"
#include "app/core/core_dependencies/common/geometry_util.h"
#include "common/log/Logger.h"

using namespace stoic;
using namespace stoic::app::core;
static const char* LOG_TAG =  "msf_loc";

namespace stoic::app::core {
void MsfLocalizationCore::GpsCallback(const std::shared_ptr<sim_localization::Gps> &msg) {
  Performance("GPS");
  gps_msg_ = *msg;
  is_gps_ready = true;
  LOG_INFO("===========GPS Driver Instance===========\n"
           "  received topic is: %s, , Msg Type is: %s,",
           FLAGS_sim_gps_topic.c_str(),
           reflection<sim_localization::Gps>::fullName().c_str());
}
void MsfLocalizationCore::ImuCallback(const std::shared_ptr<sim_localization::Imu> &msg) {
  Performance("IMU");
  imu_msg_ = *msg;
  is_imu_ready = true;
  LOG_INFO("===========IMU Driver Instance===========\n"
           "  received topic is: %s, , Msg Type is: %s,",
           FLAGS_sim_imu_topic.c_str(),
           reflection<sim_localization::Imu>::fullName().c_str());
}
void MsfLocalizationCore::MapMsgCallback(const std::shared_ptr<stoic::cm::proto::common::PString> &msg) {
#if defined SD_MAP
  auto code = map_interface_->ParseMessage(
      (uint8_t *)msg->data().c_str(), msg->data().size());
  if (code != caic_map::Av3hrCode::OK) {
    LOG_ERROR("ParseMessage error code = %d", (uint8_t)code + 0);
  }
#endif
}
void MsfLocalizationCore::run() {
  static stoic::cm::Subscriber sub_gps = nh_ptr_->subscribe<::sim_localization::Gps, true>(
      FLAGS_sim_gps_topic, 1,
      [this](auto && gps_data) { GpsCallback(std::forward<decltype(gps_data)>(gps_data)); });
  static stoic::cm::Subscriber sub_imu = nh_ptr_->subscribe<::sim_localization::Imu, true>(
      FLAGS_sim_imu_topic, 1,
      [this](auto && imu_data) { ImuCallback(std::forward<decltype(imu_data)>(imu_data)); });
#if defined MW_ART_IPC || defined MW_ART_ORIN
  static stoic::cm::Subscriber sub_map = nh_ptr_->subscribe<::stoic::cm::proto::common::PString, false>(
      "/map/stream_raw", 1,
      [this](auto && map_data) { MapMsgCallback(std::forward<decltype(map_data)>(map_data)); });
#endif
  using LocType = ::caic_localization::LocalizationEstimation;
  static stoic::cm::Publisher pub_loc =
      nh_ptr_->advertise<LocType, true>(FLAGS_localization_topic, 1);
  Rate rate(100);
  while (stoic::cm::ok()) {
    caic_localization::LocalizationEstimation localization_result{};
    process(localization_result);
    if (is_gps_ready && is_imu_ready)
      pub_loc.publish<LocType, true>(localization_result);
    rate.sleep();
  }
}
bool MsfLocalizationCore::process(caic_localization::LocalizationEstimation &result) {
  if (!is_gps_ready || !is_imu_ready) {
    LOG_ERROR("Cannot find gps and imu msg!");
    return false;
  }
  double start_time = double(Timer::now_us()) * 1.0 / 1000000l;
  double gps_time = double(gps_msg_.header.stamp) * 1.0 / 1000000l;
  double time_delay = last_received_timestamp_sec_ ? (gps_time - last_received_timestamp_sec_)
                                                   : last_received_timestamp_sec_;
  if (time_delay > 0.5) {
    LOG_ERROR("GPS message time interval: %f gps_time: %f last_received_timestamp_sec_: %f",
              time_delay, gps_time, last_received_timestamp_sec_);
//     return false;
  }
  last_received_timestamp_sec_ = gps_time;
  if ((start_time - gps_time) > 0.5) {
    LOG_ERROR("GPS message is delay: %f start_time: %f gps_time: %f",
              start_time - gps_time, start_time, gps_time);
    // return false;
  }
  if (!composeLocalizationMsg(gps_msg_, imu_msg_, &result))
    return false;
  return true;
}

bool MsfLocalizationCore::composeLocalizationMsg(const sim_localization::Gps &gps_msg,
                                                    const sim_localization::Imu &imu_msg,
                                                    caic_localization::LocalizationEstimation *localization) {
  // fill header
  fillLocalizationHeader(localization);
//  localization->status = caic_localization::AbsoluteLocalizationStatus::LOCAL_BAD;
//  localization->available |=
//      caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_STATUS;

  if (is_abs_pos_valid) {
    fillLocalizationAbsPosMsg(gps_msg, imu_msg, localization);
#if defined MW_ART_IPC || defined MW_ART_ORIN
    //相对定位
    localization->relative_pose.available = caic_localization::LocalizationPose::POSE;
    localization->relative_pose.pose_info.position.x = gps_msg.gps_info.pose_info.x;
    localization->relative_pose.pose_info.position.y = gps_msg.gps_info.pose_info.y;
    localization->relative_pose.pose_info.position.z = gps_msg.gps_info.pose_info.z;
    localization->relative_pose.pose_info.quaternion.x = localization->pose.pose_info.quaternion.x;
    localization->relative_pose.pose_info.quaternion.y = localization->pose.pose_info.quaternion.y;
    localization->relative_pose.pose_info.quaternion.z = localization->pose.pose_info.quaternion.z;
    localization->relative_pose.pose_info.quaternion.w = localization->pose.pose_info.quaternion.w;
    //local state
    localization->status = caic_localization::AbsoluteLocalizationStatus::LOCAL_GOOD;
    localization->road_status = caic_localization::RoadMatchStatus::STATUS_ONROAD;
    localization->available = caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_RELATIVE_POSE
        | caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_POSE;
    //===========utm to latlon===========================//
    if (Config::CURRENT_MODE == Config::Mode::NORMAL) {
      double lat, lon;
      std::string zone = "50R";
      UTMtoLL(localization->pose.pose_info.position.x + kMapOffsetX,
              localization->pose.pose_info.position.y + kMapOffsetY, zone, lat, lon);
      localization->lla.x = lon;
      localization->lla.y = lat;
      localization->lla.z = localization->pose.pose_info.position.z;
      localization->available |=
          caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_LLA;
    }
#endif
  }
//  if (is_rel_pos_valid) {
//// ============相对定位信息============
////    bool rel_pose = fillLocalizationRelPosMsg(gps_msg, imu_msg, localization);
////    if (!rel_pose) LOG_ERROR("fill relative pose failed!!!");
//    localization->relative_pose.available = caic_localization::LocalizationPose::POSE;
//    localization->relative_pose.pose_info.position.x = gps_msg.gps_info.pose_info.x;
//    localization->relative_pose.pose_info.position.y = gps_msg.gps_info.pose_info.y;
//    localization->relative_pose.pose_info.position.z = gps_msg.gps_info.pose_info.z;
//    localization->relative_pose.pose_info.quaternion.x = localization->pose.pose_info.quaternion.x;
//    localization->relative_pose.pose_info.quaternion.y = localization->pose.pose_info.quaternion.y;
//    localization->relative_pose.pose_info.quaternion.z = localization->pose.pose_info.quaternion.z;
//    localization->relative_pose.pose_info.quaternion.w = localization->pose.pose_info.quaternion.w;
//
//    localization->available =
//        caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_RELATIVE_POSE;
//
//  }
// 添加sdmap
#if defined SD_MAP
  if (Config::CURRENT_MODE == Config::Mode::NORMAL) {
//    localization->road_status = caic_localization::RoadMatchStatus::STATUS_OFFROAD;
    if (is_abs_pos_valid) {
      caic_map::PositionX position;
      caic_map::Av3hrCode ret = map_interface_->UTM2OffsetRapid(
          localization->pose.pose_info.position.x + kMapOffsetX,
          localization->pose.pose_info.position.y + kMapOffsetY, position);
      LOG_ERROR("UTM2OffsetRapid ret = %d, x = %f, y= %f, lla = %f %f %f", (uint8_t) ret + 0,
                localization->pose.pose_info.position.x + kMapOffsetX,
                localization->pose.pose_info.position.y + kMapOffsetY, localization->lla.x,
                localization->lla.y, localization->lla.z);
      if (ret == caic_map::Av3hrCode::OK) {
        LOG_ERROR(
            "PositionX lane_id = %d, link_id = %d, path_id = %d ,link offset = %d,"
            "current_lane = %d, offset_global = %d",
            (uint64_t) position.lane_id + 0, (uint64_t) position.link_id + 0,
            (uint32_t) position.path_id + 0, (uint32_t) position.offset + 0,
            (uint8_t) position.current_lane + 0, (uint32_t) position.offset_global + 0);
        localization->lane_id = position.lane_id;
        localization->link_id = position.link_id;
        localization->path_id = position.path_id;
//        localization->road_status = caic_localization::RoadMatchStatus::STATUS_ONROAD;
        localization->road_match_confidence = 1.0;
        localization->lane_number = position.current_lane;
        localization->dis_to_link_start = position.offset / 100.0;
        localization->dis_to_global_start = position.offset_global / 100.0;
        localization->available = caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_RELATIVE_POSE
            | caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_POSE
            | caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_MSF_STATUS
            | caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_ROAD_MATCH_CONFIDENCE;
//        localization->available |=
//            caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_LANE_ID |
//            caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_LINK_ID |
//            caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_PATH_ID |
//            caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_ROAD_STATUS |
//            caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_ROAD_MATCH_CONFIDENCE |
//            caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_DIS_TO_LINK_START |
//            caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_LANE_NUMBER |
//            caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_DIS_TO_GLOBAL_START;
      }
      else {
        LOG_ERROR("[MSF_LOC] UTM2OffsetRapid ret = %d", ret);
        localization->path_id = 1;
        localization->link_id = 0;
        localization->lane_id = 0;
        localization->lane_number = 1;
        localization->dis_to_link_start = 0.0;
        localization->dis_to_global_start = 0.0;
        localization->available = caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_RELATIVE_POSE
            | caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_POSE;
      }
    }
  }
#endif
//  if (!(localization->available &
//        caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_ROAD_STATUS)) {
//    localization->road_status = caic_localization::RoadMatchStatus::STATUS_OFFROAD;
//    return false;
//  }
  return true;
}
void MsfLocalizationCore::fillLocalizationHeader(caic_localization::LocalizationEstimation *localization) {
  localization->header.seq = seq_num++;
  localization->header.stamp = Timer::now_us();
  strcpy(localization->header.module_name, "sim_msf_loc");
  strcpy(localization->header.frame_id, "map");
}
bool MsfLocalizationCore::fillLocalizationAbsPosMsg(const sim_localization::Gps &gps_msg,
                                                       const sim_localization::Imu &imu_msg,
                                                       caic_localization::LocalizationEstimation *localization) {
  // get nearest lane
  apollo::geometry::PointENU egoPose;
  egoPose.set_x(gps_msg.gps_info.pose_info.x);
  egoPose.set_y(gps_msg.gps_info.pose_info.y);
  egoPose.set_z(gps_msg.gps_info.pose_info.z);
  double targetZ = 0;
  std::string lanIdStr = "";
  std::string hdmap_file = "/opt/usr/t3caic/hdmap/park/base_map.bin";
  bool hdmap_flag = apollo::hdmap::HDMapUtil::ReloadMaps(hdmap_file);
  if (hdmap_flag) {
    double dis_range = 10.0;
    double egoHeading  = imu_msg.imu_info.heading;
    apollo::geometry::PointENU nearestLanePt;
    if(!getNearestLane(egoPose, egoHeading, dis_range, lanIdStr, nearestLanePt))
    {
      LOG_ERROR("Cannot find lane! egoPose:%.3f, %.3f, %.1f", egoPose.x(), egoPose.y(), egoPose.z());
      return false;
    }
    targetZ = nearestLanePt.z();
  } else {
    targetZ = 0;
  }
  
  // rpy->Quaternion
  apollo::math::EulerAnglesZXYd quaternion(imu_msg.imu_info.angle.roll,
                                           imu_msg.imu_info.angle.pitch,
                                           imu_msg.imu_info.angle.yaw - M_PI_2);
  Eigen::Quaterniond q(
      quaternion.ToQuaternion().w(), quaternion.ToQuaternion().x(),
      quaternion.ToQuaternion().y(), quaternion.ToQuaternion().z());
  Eigen::Matrix3d roation_matrix = q.toRotationMatrix();
  // pose info
  localization->pose.pose_info.position.x = gps_msg.gps_info.pose_info.x;
  localization->pose.pose_info.position.y = gps_msg.gps_info.pose_info.y;
  localization->pose.pose_info.position.z = targetZ; //gps_msg.gps_info.pose_info.z;
  localization->pose.pose_info.quaternion.w = quaternion.ToQuaternion().w();
  localization->pose.pose_info.quaternion.x = quaternion.ToQuaternion().x();
  localization->pose.pose_info.quaternion.y = quaternion.ToQuaternion().y();
  localization->pose.pose_info.quaternion.z = quaternion.ToQuaternion().z();
//  localization->pose.pose_info.available = caic_std::PoseWithCovariance::POSITION |
//      caic_std::PoseWithCovariance::ORIENTATION |
//      caic_std::PoseWithCovariance::COVARINACE;
  // heading
  double heading = quaternionToHeading(
      quaternion.ToQuaternion().w(), quaternion.ToQuaternion().x(),
      quaternion.ToQuaternion().y(), quaternion.ToQuaternion().z());
  localization->pose.heading = heading;
  // linear velocity info
  Eigen::Vector3d rel_vel(gps_msg.gps_info.linear_velocity_info.x,
                          gps_msg.gps_info.linear_velocity_info.y,
                          gps_msg.gps_info.linear_velocity_info.z);
  Eigen::Vector3d world_vel = roation_matrix * rel_vel;
  localization->pose.linear_velocity_info.linear_velocity.x = world_vel[0];
  localization->pose.linear_velocity_info.linear_velocity.y = world_vel[1];
  localization->pose.linear_velocity_info.linear_velocity.z = world_vel[2];
  localization->pose.linear_velocity_info.linear_velocity_covariance.data[0] = 1e-3;
  localization->pose.linear_velocity_info.linear_velocity_covariance.data[4] = 1e-3;
  localization->pose.linear_velocity_info.linear_velocity_covariance.data[8] = 1e-3;
//  localization->pose.linear_velocity_info.available =
//      caic_std::LinearVelocityWithCovariance::VALUE |
//          caic_std::LinearVelocityWithCovariance::LINEAR_VELOCITY_COVARIANCE;
  // linear acceleration
  // orig -> world
  Eigen::Vector3d rel_acc(imu_msg.imu_info.linear_acceleration_vrf_info.x,
                          imu_msg.imu_info.linear_acceleration_vrf_info.y,
                          imu_msg.imu_info.linear_acceleration_vrf_info.z);
  Eigen::Vector3d world_acc = roation_matrix * rel_acc;
  localization->pose.linear_acceleration_info.linear_acceleration.x = world_acc[0];
  localization->pose.linear_acceleration_info.linear_acceleration.y = world_acc[1];
  localization->pose.linear_acceleration_info.linear_acceleration.z = world_acc[2];
  localization->pose.linear_acceleration_info.linear_acceleration_covariance.data[0] = 1e-3;
  localization->pose.linear_acceleration_info.linear_acceleration_covariance.data[4] = 1e-3;
  localization->pose.linear_acceleration_info.linear_acceleration_covariance.data[8] = 1e-3;
//  localization->pose.linear_acceleration_info.available =
//      caic_std::LinearAccelerationWithCovariance::VALUE |
//          caic_std::LinearAccelerationWithCovariance::LINEAR_ACCELERATION_COVARIANCE;
  localization->pose.linear_acceleration_vrf_info.linear_acceleration.x = imu_msg.imu_info.linear_acceleration_vrf_info.x;
  localization->pose.linear_acceleration_vrf_info.linear_acceleration.y = imu_msg.imu_info.linear_acceleration_vrf_info.y;
  localization->pose.linear_acceleration_vrf_info.linear_acceleration.z = imu_msg.imu_info.linear_acceleration_vrf_info.z;
  localization->pose.linear_acceleration_vrf_info.linear_acceleration_covariance.data[0] = 1e-3;
  localization->pose.linear_acceleration_vrf_info.linear_acceleration_covariance.data[4] = 1e-3;
  localization->pose.linear_acceleration_vrf_info.linear_acceleration_covariance.data[8] = 1e-3;
//  localization->pose.linear_acceleration_vrf_info.available =
//      caic_std::LinearAccelerationWithCovariance::VALUE |
//          caic_std::LinearAccelerationWithCovariance::LINEAR_ACCELERATION_COVARIANCE;
  // angular velocity
  Eigen::Vector3d rel_avel(imu_msg.imu_info.angular_velocity_vrf_info.x,
                           imu_msg.imu_info.angular_velocity_vrf_info.y,
                           imu_msg.imu_info.angular_velocity_vrf_info.z);
  Eigen::Vector3d world_avel = roation_matrix * rel_avel;
  localization->pose.angular_velocity_info.angular_velocity.x = world_avel[0];
  localization->pose.angular_velocity_info.angular_velocity.y = world_avel[1];
  localization->pose.angular_velocity_info.angular_velocity.z = world_avel[2];
  localization->pose.angular_velocity_info.angular_velocity_covariance.data[0] = 1e-3;
  localization->pose.angular_velocity_info.angular_velocity_covariance.data[4] = 1e-3;
  localization->pose.angular_velocity_info.angular_velocity_covariance.data[8] = 1e-3;
//  localization->pose.angular_velocity_info.available =
//      caic_std::AngularVelocityWithCovariance::VALUE |
//          caic_std::AngularVelocityWithCovariance::ANGULAR_VELOCITY_COVARIANCE;
  localization->pose.angular_velocity_vrf_info.angular_velocity.x = imu_msg.imu_info.angular_velocity_vrf_info.x;
  localization->pose.angular_velocity_vrf_info.angular_velocity.y = imu_msg.imu_info.angular_velocity_vrf_info.y;
  localization->pose.angular_velocity_vrf_info.angular_velocity.z = imu_msg.imu_info.angular_velocity_vrf_info.z;
  localization->pose.angular_velocity_vrf_info.angular_velocity_covariance.data[0] = 1e-3;
  localization->pose.angular_velocity_vrf_info.angular_velocity_covariance.data[4] = 1e-3;
  localization->pose.angular_velocity_vrf_info.angular_velocity_covariance.data[8] = 1e-3;
//  localization->pose.angular_velocity_vrf_info.available =
//      caic_std::AngularVelocityWithCovariance::VALUE |
//          caic_std::AngularVelocityWithCovariance::ANGULAR_VELOCITY_COVARIANCE;
  // angle
  localization->pose.angle.roll  = imu_msg.imu_info.angle.roll;
  localization->pose.angle.pitch = imu_msg.imu_info.angle.pitch;
  localization->pose.angle.yaw   = imu_msg.imu_info.angle.yaw;
  localization->pose.available = caic_localization::LocalizationPose::POSE;
//  localization->pose.available = caic_localization::LocalizationPose::POSE |
//      caic_localization::LocalizationPose::LINEAR_VELOCITY |
//      caic_localization::LocalizationPose::LINEAR_ACCELERATION |
//      caic_localization::LocalizationPose::ANGLE_VELOCITY |
//      caic_localization::LocalizationPose::LINEAR_ACCELERATION_VRF |
//      caic_localization::LocalizationPose::ANGLE_VELOCITY_VRF |
//      caic_localization::LocalizationPose::HEADING |
//      caic_localization::LocalizationPose::ANGLE;
  if (hdmap_flag)
    localization->localization_level = abs(atoi(lanIdStr.c_str()));
  LOG_INFO("========LocalizationAbsPosMsg========\n"
            " pos x/y/z= %.3f %.3f %.3f, heading= %.3f, vel x/y/z= %.3f %.3f %.3f",
            gps_msg.gps_info.pose_info.x, gps_msg.gps_info.pose_info.y, gps_msg.gps_info.pose_info.z,
            heading, gps_msg.gps_info.linear_velocity_info.x, gps_msg.gps_info.linear_velocity_info.y, gps_msg.gps_info.linear_velocity_info.z);
}
bool MsfLocalizationCore::fillLocalizationRelPosMsg(const sim_localization::Gps &gps_msg,
                                                       const sim_localization::Imu &imu_msg,
                                                       caic_localization::LocalizationEstimation *localization) {
    current_time = Timer::now();
    LOG_INFO("[sim-msf-loc] 当前时间戳: %ld",current_time);
    LOG_INFO("[sim-msf-loc] 接收的序列号: %u,时间戳: %ld",gps_msg.header.seq,gps_msg.header.stamp);
    LOG_INFO("[sim-msf-loc] 接收的序列号: %u,时间戳: %ld",imu_msg.header.seq,imu_msg.header.stamp);
    // relative localization data
    is_rel_pos_valid = getRelPose(current_time);
    // 位置
    localization->relative_pose.pose_info.position.x = rel_loc_data_.pos_wi_.x();
    localization->relative_pose.pose_info.position.y = rel_loc_data_.pos_wi_.y();
    localization->relative_pose.pose_info.position.z = rel_loc_data_.pos_wi_.z();
    // 四元素
    localization->relative_pose.pose_info.quaternion.w = rel_loc_data_.qt.w();
    localization->relative_pose.pose_info.quaternion.x = rel_loc_data_.qt.x();
    localization->relative_pose.pose_info.quaternion.y = rel_loc_data_.qt.y();
    localization->relative_pose.pose_info.quaternion.z = rel_loc_data_.qt.z();
    localization->relative_pose.pose_info.available = caic_std::PoseWithCovariance::POSITION |
        caic_std::PoseWithCovariance::ORIENTATION |
        caic_std::PoseWithCovariance::COVARINACE;
    // linear velocity info
    localization->relative_pose.linear_velocity_info.linear_velocity.x = rel_loc_data_.vel_wi_.x();
    localization->relative_pose.linear_velocity_info.linear_velocity.y = rel_loc_data_.vel_wi_.y();
    localization->relative_pose.linear_velocity_info.linear_velocity.z = rel_loc_data_.vel_wi_.z();
    localization->relative_pose.linear_velocity_info.available =
        caic_std::LinearVelocityWithCovariance::VALUE |
            caic_std::LinearVelocityWithCovariance::LINEAR_VELOCITY_COVARIANCE;
    //heading
    localization->relative_pose.heading = rel_loc_data_.heading;
    // angle
    localization->relative_pose.angle.roll  = rel_loc_data_.rpy_wi_.x();
    localization->relative_pose.angle.pitch = rel_loc_data_.rpy_wi_.y();
    localization->relative_pose.angle.yaw   = rel_loc_data_.rpy_wi_.z();
    localization->relative_pose.available = caic_localization::LocalizationPose::POSE |
        caic_localization::LocalizationPose::HEADING |
        caic_localization::LocalizationPose::ANGLE;
  LOG_INFO("========LocalizationRelPosMsg========\n"
           " pos x/y/z= %.3f %.3f %.3f, heading= %.3f, vel x/y/z= %.3f %.3f %.3f",
           localization->relative_pose.pose_info.position.x, localization->relative_pose.pose_info.position.y, localization->relative_pose.pose_info.position.z,
           localization->relative_pose.heading, localization->relative_pose.linear_velocity_info.linear_velocity.x,
           localization->relative_pose.linear_velocity_info.linear_velocity.y, localization->relative_pose.linear_velocity_info.linear_velocity.z);

  return true;
}
bool MsfLocalizationCore::getRelPose(long time) {
  LocalizationState tmp_state;
  Eigen::Vector3d abs_pos(gps_msg_.gps_info.pose_info.x,
                          gps_msg_.gps_info.pose_info.y,
                          gps_msg_.gps_info.pose_info.z);
  Eigen::Vector3d abs_vel(gps_msg_.gps_info.linear_velocity_info.x,
                          gps_msg_.gps_info.linear_velocity_info.y,
                          gps_msg_.gps_info.linear_velocity_info.z);
  Eigen::Vector3d abs_acc(imu_msg_.imu_info.linear_acceleration_vrf_info.x,
                          imu_msg_.imu_info.linear_acceleration_vrf_info.y,
                          imu_msg_.imu_info.linear_acceleration_vrf_info.z);
  Eigen::Vector3d abs_rpy(imu_msg_.imu_info.angle.roll,
                          imu_msg_.imu_info.angle.pitch,
                          imu_msg_.imu_info.angle.yaw);
  Eigen::Vector3d abs_avel(imu_msg_.imu_info.angular_velocity_vrf_info.x,
                          imu_msg_.imu_info.angular_velocity_vrf_info.y,
                          imu_msg_.imu_info.angular_velocity_vrf_info.z);
  if (labs(time - gps_msg_.header.stamp) < 1e5 && labs(time - imu_msg_.header.stamp) < 1e5) {
    LOG_INFO("获取第一帧定位数据！！！");
    Eigen::Vector3d start_pos, start_vel, start_acc, start_rpy, start_avel;
    start_pos = tmp_state.Rwi_ * abs_pos;
    start_vel = tmp_state.Rwi_ * abs_vel;
    start_acc = tmp_state.Rwi_ * abs_acc;
    start_rpy = tmp_state.Rwi_ * abs_rpy;
    start_avel = tmp_state.Rwi_ * abs_avel;
    tmp_state.pos_wi_ -= start_pos;
    tmp_state.vel_wi_ -= start_vel;
    tmp_state.acc_wi_ -= start_acc;
    tmp_state.rpy_wi_ -= start_rpy;
    tmp_state.avel_wi_ -= start_avel;
    //四元素
    apollo::math::EulerAnglesZXYd start_quat(imu_msg_.imu_info.angle.roll,
                                             imu_msg_.imu_info.angle.pitch,
                                             imu_msg_.imu_info.angle.yaw - M_PI_2);
    Eigen::Quaterniond start_qt(start_quat.ToQuaternion().w(), start_quat.ToQuaternion().x(),
                                start_quat.ToQuaternion().y(), start_quat.ToQuaternion().z());
    start_qt.normalize();
    apollo::math::EulerAnglesZXYd quat(tmp_state.rpy_wi_.x(),
                                       tmp_state.rpy_wi_.y(),
                                       tmp_state.rpy_wi_.z() - M_PI_2);
    Eigen::Quaterniond qt(quat.ToQuaternion().w(), quat.ToQuaternion().x(),
                          quat.ToQuaternion().y(), quat.ToQuaternion().z());
    qt.normalize();
    tmp_state.qt.w() = qt.w() - start_qt.w();
    tmp_state.qt.x() = qt.x() - start_qt.x();
    tmp_state.qt.y() = qt.y() - start_qt.y();
    tmp_state.qt.z() = qt.z() - start_qt.z();
    // heading
    tmp_state.heading = quaternionToHeading(tmp_state.qt.w(), tmp_state.qt.x(),
                                            tmp_state.qt.y(), tmp_state.qt.z());
  }
  rel_loc_data_ = tmp_state;
  rel_loc_data_.timestamp = tmp_state.timestamp;
  return true;
}
double MsfLocalizationCore::quaternionToHeading(const double qw, const double qx, const double qy, const double qz) {
  apollo::math::EulerAnglesZXYd euler_angles(qw, qx, qy, qz);
  return apollo::math::NormalizeAngle(euler_angles.yaw() + M_PI_2);
}
bool MsfLocalizationCore::getNearestLane(const apollo::geometry::PointENU& ego_position, double ego_heading, double distance_range, std::string& laneId,
                               apollo::geometry::PointENU& nearestLanePt)
{
  const apollo::hdmap::HDMap* hdmap = apollo::hdmap::HDMapUtil::BaseMapPtr();
  if(nullptr == hdmap)
  {
    return false;
  }
  std::vector<apollo::hdmap::LaneInfoConstPtr> lanes;
  hdmap->GetLanes(ego_position, distance_range, &lanes);
  // SL_ERROR("HDMap lanes %d", lanes.size());
  if(lanes.size() <= 0)
  {
    return false;
  }
  apollo::geometry::PointENU egoHeadVec;
  egoHeadVec.set_x(cos(ego_heading));
  egoHeadVec.set_y(sin(ego_heading));
  // lanes
  std::string targetLaneId = "";
  std::string targetLaneId_direct = "";
  apollo::geometry::PointENU targetPntProject;
  apollo::geometry::PointENU targetPntProject_direct;
  double dMinDis = -1.0;
  double dMinDis_direct = -1.0;
  for (apollo::hdmap::LaneInfoConstPtr& lane_ptr : lanes) {
    auto id = lane_ptr->id().id();
    double dMinDis_inner = -1.0;
    apollo::geometry::PointENU headVector_inner;
    apollo::geometry::PointENU pntProject_inner;
    for (const auto &curve : lane_ptr->lane().central_curve().segment()) {
      apollo::geometry::PointENU headVector;
      apollo::geometry::PointENU nearestPt;
      double disToLane = getNearestDistance(ego_position, curve.line_segment(), distance_range, headVector, nearestPt);
      if(disToLane < dMinDis_inner || dMinDis_inner < 0.0)
      {
        dMinDis_inner = disToLane;
        headVector_inner.set_x(headVector.x());
        headVector_inner.set_y(headVector.y());
        pntProject_inner = nearestPt;
      }
    }
    if(dMinDis_inner < dMinDis || dMinDis < 0.0)
    {
      dMinDis = dMinDis_inner;
      targetLaneId = id;
      targetPntProject = pntProject_inner;
    }
    // should same direct
    if(!isSameDirect(egoHeadVec,headVector_inner, 90.0))
    {
      continue;
    }
    if(dMinDis_inner < dMinDis_direct || dMinDis_direct < 0.0)
    {
      dMinDis_direct = dMinDis_inner;
      targetLaneId_direct = id;
      targetPntProject_direct = pntProject_inner;
    }
  }
  laneId = (dMinDis_direct > 0.0)?targetLaneId_direct: targetLaneId;
  nearestLanePt = (dMinDis_direct > 0.0)?targetPntProject_direct:targetPntProject;
  return true;
}
} // namespace stoic::app::core
