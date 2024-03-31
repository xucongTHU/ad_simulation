
//
// Created by xucong on 24-1-3.
//
#include "surround_vision_detector.h"
#include "yaml-cpp/yaml.h"
#include "common/geometry_util.h"
#include "third_party/apollo/math/euler_angles_zxy.h"
#include "third_party/apollo/proto/hdmap/map.pb.h"
#include "third_party/apollo/hdmap/hdmap.h"
#include "third_party/apollo/hdmap/hdmap_util.h"
#include "common/config.h"
#include "app/core/sim_perception/common/tf_convert.h"

using namespace stoic;
using namespace stoic::app::core;

namespace stoic::app::core {
bool SurroundVisionDetector::getLocateParkSpace(const apollo::geometry::PointENU &ego_position,
                                                std::string &parkspaceId,
                                                apollo::geometry::PointENU &leftTopPt,
                                                apollo::geometry::PointENU &rightTopPt) {
  const apollo::hdmap::HDMap* hdmap = apollo::hdmap::HDMapUtil::BaseMapPtr();
  if(nullptr == hdmap)
  {
    return false;
  }
  std::vector<apollo::hdmap::ParkingSpaceInfoConstPtr> parking_spaces;
  hdmap->GetParkingSpaces(ego_position, 0.1, &parking_spaces);
  if(parking_spaces.size() <= 0)
  {
    return false;
  }
  for (apollo::hdmap::ParkingSpaceInfoConstPtr& parking_space_ptr : parking_spaces) {
    parkspaceId = parking_space_ptr->id().id();
    const auto &repeatePoints = parking_space_ptr->parking_space().polygon().point();
    if(repeatePoints.size() < 4)
    {
      return false;
    }
    //lefttop-index(3), righttop-index(2)
    leftTopPt.set_x(repeatePoints.Get(3).x());
    leftTopPt.set_y(repeatePoints.Get(3).y());
    leftTopPt.set_z(repeatePoints.Get(3).z());
    rightTopPt.set_x(repeatePoints.Get(2).x());
    rightTopPt.set_y(repeatePoints.Get(2).y());
    rightTopPt.set_z(repeatePoints.Get(2).z());
    break;
  }
  return true;
}
void SurroundVisionDetector::surroundVisionCallback(const std::shared_ptr<sim_perception::PerceptionSurround> &msg) {
  if (msg->parkingspace_result.parking_spaces_size == 0 ) {
    LOG_ERROR("[SurroundVision] input parking space msg info error, skip!");
    return;
  }
  else {
    perc_surround_ = msg;
    is_surround_vision_ready = true;
    LOG_INFO("[SurroundVision] received topic is: %s, Msg Type is: %s,",
             "/stoic/surround/parkspace",
             reflection<sim_perception::PerceptionSurround>::fullName().c_str());
  }
}
void SurroundVisionDetector::planningStateCallback(const std::shared_ptr<caic_interaction::PlanningState> &msg) {
  caic_interaction::DriveMode switchDriveMode = msg->switch_mode.mode;
  is_plan_ready = true;
  LOG_INFO("planningStateCallback, state:%d, space_id:%d, left_top_x: %.3f, left_top_y: %.3f, right_top_x: %.3f, right_top_y: %.3f", (int)switchDriveMode,
           msg->switch_mode.parking_space_id, msg->switch_mode.parking_space_left_top.x, msg->switch_mode.parking_space_left_top.y,
           msg->switch_mode.parking_space_right_top.x, msg->switch_mode.parking_space_right_top.y);
  std::string loc_state_topic = "/smartcar/localization/state";
  static ::stoic::cm::Publisher pub_loc_state =
      nh_ptr_->advertise<caic_perception::PerceptionObjects, true>(loc_state_topic, 1);
  switch(switchDriveMode)
  {
    case caic_interaction::DriveMode::CRUISE_IN:
    case caic_interaction::DriveMode::CRUISE_OUT:
    {
      caic_interaction::LocalizationState locState;
      strncpy(locState.header.module_name, "loc_state", STD_MODULE_NAME_SIZE);
      locState.header.stamp = Timer::now_us();
      locState.state = caic_interaction::LocalizationState::State::CRUISE_SUCCESS;
      locState.switch_mode.mode = switchDriveMode;
      pub_loc_state.publish<caic_interaction::LocalizationState, true>(locState);
      LOG_INFO("LocalizationState pub, mode: CRUISE, state: %d", (int)locState.state);
    }
      break;
    case caic_interaction::DriveMode::PARK_IN:
    {
      caic_interaction::LocalizationState locState;
      strncpy(locState.header.module_name, "loc_state", STD_MODULE_NAME_SIZE);
      locState.header.stamp = Timer::now_us();
      locState.state = caic_interaction::LocalizationState::State::PARK_SUCCESS;
      locState.switch_mode.mode = switchDriveMode;
      locState.switch_mode.parking_space_id = msg->switch_mode.parking_space_id;
      //check the park with special ID if match the exist park space
      double egoX = pose_manager->pose.pose_info.position.x;
      double egoY = pose_manager->pose.pose_info.position.y;
      double egoHeading = pose_manager->pose.heading - M_PI_2;
      LOG_INFO("LocalizationState pub, mode: PARK_IN, egoX: %.3f, egoY: %.3f, egoHeading: %.3f", egoX, egoY, egoHeading);
      // vcs pt to map pt, rotate -Heading, then add egoXY
      double rotLeftX=0.0, rotLeftY=0.0, rotRightX=0.0, rotRightY=0.0;
      rotate(msg->switch_mode.parking_space_left_top.x, msg->switch_mode.parking_space_left_top.y, egoHeading, rotLeftX, rotLeftY);
      rotate(msg->switch_mode.parking_space_right_top.x, msg->switch_mode.parking_space_right_top.y, egoHeading, rotRightX, rotRightY);
      locState.parking_space_left_top.x = rotLeftX + egoX;
      locState.parking_space_left_top.y = rotLeftY + egoY;
      locState.parking_space_right_top.x = rotRightX + egoX;
      locState.parking_space_right_top.y = rotRightY + egoY;
      pub_loc_state.publish<caic_interaction::LocalizationState, true>(locState);
      LOG_INFO("LocalizationState pub, mode: PARK_IN, state: %d, space_id:%d, left_top_x: %.3f, left_top_y: %.3f, right_top_x: %.3f, right_top_y: %.3f", (int)locState.state,
              locState.switch_mode.parking_space_id, locState.parking_space_left_top.x, locState.parking_space_left_top.y,
              locState.parking_space_right_top.x,  locState.parking_space_right_top.y);
    }
      break;
    case caic_interaction::DriveMode::PARK_OUT:
    {
      //find park space by location from basemap
      apollo::geometry::PointENU egoPose;
      egoPose.set_x(pose_manager->pose.pose_info.position.x);
      egoPose.set_y(pose_manager->pose.pose_info.position.y);
      egoPose.set_z(pose_manager->pose.pose_info.position.z);
      std::string parkspaceId="";
      apollo::geometry::PointENU leftTopPt;
      apollo::geometry::PointENU rightTopPt;
      if(!getLocateParkSpace(egoPose, parkspaceId, leftTopPt, rightTopPt))
      {
        LOG_ERROR("Cannot find parkspace! egoPose:%.3f, %.3f, %.1f",
                 pose_manager->pose.pose_info.position.x, pose_manager->pose.pose_info.position.y, pose_manager->pose.pose_info.position.z);
        return;
      }
      caic_interaction::LocalizationState locState;
      strncpy(locState.header.module_name, "loc_state", STD_MODULE_NAME_SIZE);
      locState.header.stamp = Timer::now_us();
      locState.state = caic_interaction::LocalizationState::State::PARK_SUCCESS;
      locState.switch_mode.mode = switchDriveMode;
      locState.switch_mode.parking_space_id = abs(atoi(parkspaceId.c_str()));
      locState.parking_space_left_top.x = leftTopPt.x();
      locState.parking_space_left_top.y = leftTopPt.y();
      locState.parking_space_right_top.x = rightTopPt.x();
      locState.parking_space_right_top.y = rightTopPt.y();
      pub_loc_state.publish<caic_interaction::LocalizationState, true>(locState);
      LOG_INFO("LocalizationState pub, mode: PARK_OUT, state: %d, space_id:%d, left_top_x: %.3f, left_top_y: %.3f, right_top_x: %.3f, right_top_y: %.3f",
              (int)locState.state,
              locState.switch_mode.parking_space_id, locState.parking_space_left_top.x, locState.parking_space_left_top.y,
              locState.parking_space_right_top.x,  locState.parking_space_right_top.y);
    }
      break;
    case caic_interaction::DriveMode::START_RAS:
    case caic_interaction::DriveMode::PILOT:
    case caic_interaction::DriveMode::INIT:
    {
      caic_interaction::LocalizationState locState;
      strncpy(locState.header.module_name, "loc_state", STD_MODULE_NAME_SIZE);
      locState.header.stamp = Timer::now_us();
      locState.state = caic_interaction::LocalizationState::State::CRUISE_SUCCESS;
      locState.switch_mode.mode = switchDriveMode;
      pub_loc_state.publish<caic_interaction::LocalizationState, true>(locState);
      LOG_INFO("LocalizationState pub, mode: START_RAS/PILOT/INIT, state: %d", (int)locState.state);
    }
      break;
    default:
    {
      caic_interaction::LocalizationState locState;
      strncpy(locState.header.module_name, "loc_state", STD_MODULE_NAME_SIZE);
      locState.header.stamp = Timer::now_us();
      locState.state = caic_interaction::LocalizationState::State::CRUISE_SUCCESS;
      locState.switch_mode.mode = switchDriveMode;
      pub_loc_state.publish<caic_interaction::LocalizationState, true>(locState);
      LOG_INFO("LocalizationState pub, mode: UNKNOWN, state: %d", (int)locState.state);
    }
      break;
  }
}
void SurroundVisionDetector::poseStateCallback(const std::shared_ptr<caic_localization::LocalizationEstimation> &msg) {
  // if ( (!msg->available &&
  //     msg->pose.available != caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_POSE) ||
  //     (!msg->available &&
  //         msg->pose.available != caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_RELATIVE_POSE)) {
  //   //
  //   LOG_ERROR("[SurroundVisionDetector] input loc msg info error, skip！！！");
  // }
  // else {
  //   pose_manager = msg;
  //   is_pos_ready = true;
  //   LOG_INFO("[LocalizationAbsPos] pos x/y/z= %.3f %.3f %.3f, heading= %.3f, velocity x/y/z= %.3f %.3f %.3f",
  //            msg->pose.pose_info.position.x, msg->pose.pose_info.position.y, msg->pose.pose_info.position.z,
  //            msg->pose.heading,
  //            msg->pose.linear_velocity_info.linear_velocity.x,
  //            msg->pose.linear_velocity_info.linear_velocity.y,
  //            msg->pose.linear_velocity_info.linear_velocity.z);
  // }
  pose_manager = msg;
    is_pos_ready = true;
    LOG_INFO("[LocalizationAbsPos] pos x/y/z= %.3f %.3f %.3f, heading= %.3f, velocity x/y/z= %.3f %.3f %.3f",
             msg->pose.pose_info.position.x, msg->pose.pose_info.position.y, msg->pose.pose_info.position.z,
             msg->pose.heading,
             msg->pose.linear_velocity_info.linear_velocity.x,
             msg->pose.linear_velocity_info.linear_velocity.y,
             msg->pose.linear_velocity_info.linear_velocity.z);
}
void SurroundVisionDetector::convert2CaicMsgs(const std::shared_ptr<sim_perception::PerceptionSurround> &output,
                                              std::shared_ptr<caic_world::BigFusion> &msg) {
  // freespace
  freespace_->Postprocess(output->detected_objects, freespace_result_);
  int fs_point_size = 0;
  for (int angle = 0; angle < 360; angle++) {
    caic_world::FreespacePoint freespace_point;
    freespace_point.vcs_point.x = freespace_result_.fs_points_[angle].vcs_x;
    freespace_point.vcs_point.y = freespace_result_.fs_points_[angle].vcs_y;
    freespace_point.type = caic_world::FreespacePointType::BACKGROUND; //
    fs_point_size++;
    msg->free_spaces.freespace_points[angle] = freespace_point;
//    LOG_ERROR("[SurroundVision]: 检测到的可行驶域点集合：(%.2f, %.2f)",
//              freespace_point.vcs_point.x, freespace_point.vcs_point.y);
  }
  msg->free_spaces.freespace_points_size = fs_point_size;
  LOG_INFO("[SurroundVision]: 检测到的可行驶域点数量：%d", fs_point_size);

  // hdmap parkspace
  apollo::geometry::PointENU egoPose;
  egoPose.set_x(pose_manager->pose.pose_info.position.x);
  egoPose.set_y(pose_manager->pose.pose_info.position.y);
  egoPose.set_z(pose_manager->pose.pose_info.position.z);
  std::string parkspaceId="";
  const apollo::hdmap::HDMap* hdmap = apollo::hdmap::HDMapUtil::BaseMapPtr();
  if(nullptr == hdmap) return;
  std::vector<apollo::hdmap::ParkingSpaceInfoConstPtr> parking_spaces;
  hdmap->GetParkingSpaces(egoPose, 6.24, &parking_spaces);
  if(parking_spaces.size() <= 0) return;
  
  // map->vcs
  Eigen::Quaterniond q(pose_manager->pose.pose_info.quaternion.w, pose_manager->pose.pose_info.quaternion.x,
                       pose_manager->pose.pose_info.quaternion.y, pose_manager->pose.pose_info.quaternion.z);
  Eigen::Vector3d t(pose_manager->pose.pose_info.position.x,
                    pose_manager->pose.pose_info.position.y,
                    pose_manager->pose.pose_info.position.z);
  manif::SE3d pose(manif::SE3d(t, q));
  manif::SE3d TX_M_I = pose.inverse();
  manif::SE3d TX_M_V = TX_I_V*TX_M_I;
  M2VTransform(pose, TX_M_V);    
  Eigen::Matrix3d TX_M_V_rotation =  TX_M_V.rotation();
  Eigen::Vector3d space_vcs_left_up, space_vcs_left_udown, space_vcs_right_up, space_vcs_right_down;
  int ps_size = 0; 
  for (apollo::hdmap::ParkingSpaceInfoConstPtr& parking_space_ptr : parking_spaces) {
    caic_world::ParkingSpace parking;
    parkspaceId = parking_space_ptr->id().id();
      
    const auto &repeatePoints = parking_space_ptr->parking_space().polygon().point();
    if(repeatePoints.size() < 4) return;
    //lefttop-index(3), righttop-index(2)
    parking.track_id = abs(atoi(parkspaceId.c_str()));
    // parkingspace points(map->vcs)
    Eigen::Vector3d space_map_left_up(repeatePoints.Get(3).x(), repeatePoints.Get(3).y(), 0);
    Eigen::Vector3d space_map_left_udown(repeatePoints.Get(0).x(), repeatePoints.Get(0).y(), 0);
    Eigen::Vector3d space_map_right_up(repeatePoints.Get(2).x(), repeatePoints.Get(2).y(), 0);
    Eigen::Vector3d space_map_right_down(repeatePoints.Get(1).x(), repeatePoints.Get(1).y(), 0);
    space_vcs_left_up = TX_M_V.act(space_map_left_up);
    space_vcs_left_udown = TX_M_V.act(space_map_left_udown);
    space_vcs_right_up = TX_M_V.act(space_map_right_up);
    space_vcs_right_down = TX_M_V.act(space_map_right_down);
    parking.space_vcs.parkingspace_left_up_point.x = space_vcs_left_up.x();
    parking.space_vcs.parkingspace_left_up_point.y = space_vcs_left_up.y();
    parking.space_vcs.parkingspace_left_down_point.x = space_vcs_left_udown.x();
    parking.space_vcs.parkingspace_left_down_point.y = space_vcs_left_udown.y();
    parking.space_vcs.parkingspace_right_up_point.x = space_vcs_right_up.x();
    parking.space_vcs.parkingspace_right_up_point.y = space_vcs_right_up.y();
    parking.space_vcs.parkingspace_right_down_point.x = space_vcs_right_down.x();
    parking.space_vcs.parkingspace_right_down_point.y = space_vcs_right_down.y();
    parking.space_vcs.angle1 = std::atan2(
        parking.space_vcs.parkingspace_left_down_point.y - parking.space_vcs.parkingspace_left_up_point.y,
        parking.space_vcs.parkingspace_left_down_point.x - parking.space_vcs.parkingspace_left_up_point.x);
    parking.space_vcs.angle2 = std::atan2(
        parking.space_vcs.parkingspace_right_down_point.y - parking.space_vcs.parkingspace_right_up_point.y,
        parking.space_vcs.parkingspace_right_down_point.x - parking.space_vcs.parkingspace_right_up_point.x);
    
    msg->park_spaces.parking_space[ps_size] = parking;  
    LOG_ERROR("[SurroundVision]: parking space id：%d", parking.track_id);
    LOG_ERROR("[SurroundVision]: 角点坐标left-up（%.2f, %.2f）, 角点坐标left-down（%.2f, %.2f）, "
              "角点坐标right-up（%.2f, %.2f）, 角点坐标right-down（%.2f, %.2f）",
              parking.space_vcs.parkingspace_left_up_point.x, parking.space_vcs.parkingspace_left_up_point.y,
              parking.space_vcs.parkingspace_left_down_point.x, parking.space_vcs.parkingspace_left_down_point.y,
              parking.space_vcs.parkingspace_right_up_point.x, parking.space_vcs.parkingspace_right_up_point.y,
              parking.space_vcs.parkingspace_right_down_point.x, parking.space_vcs.parkingspace_right_down_point.y);
    ps_size++;
      
  }
  msg->park_spaces.parking_space_size = ps_size;
  LOG_INFO("[SurroundVision]: 检测到的车位数量：%d", msg->park_spaces.parking_space_size);


  // parkingspace
//   int ps_size;
//   for (size_t i = 0; i < output->parkingspace_result.parking_spaces_size; i++) {
//     caic_world::ParkingSpace parking;
//     parking.track_id = output->parkingspace_result.parking_spaces[i].track_id;
//     auto parkingspace = output->parkingspace_result.parking_spaces[i];
//     parking.type = caic_world::ParkSlotType::LINE_MARK_VERTICAL;
//     parking.occu = 0;//parkingspace.occu;
//     // corner list
//     // vcs
//     parking.space_vcs.parkingspace_left_up_point.x = parkingspace.space_vcs.parkingspace_left_up_point.x;
//     parking.space_vcs.parkingspace_left_up_point.y = parkingspace.space_vcs.parkingspace_left_up_point.y;
//     parking.space_vcs.parkingspace_left_down_point.x = parkingspace.space_vcs.parkingspace_left_down_point.x;
//     parking.space_vcs.parkingspace_left_down_point.y = parkingspace.space_vcs.parkingspace_left_down_point.y;
//     parking.space_vcs.parkingspace_right_up_point.x = parkingspace.space_vcs.parkingspace_right_up_point.x;
//     parking.space_vcs.parkingspace_right_up_point.y = parkingspace.space_vcs.parkingspace_right_up_point.y;
//     parking.space_vcs.parkingspace_right_down_point.x = parkingspace.space_vcs.parkingspace_right_down_point.x;
//     parking.space_vcs.parkingspace_right_down_point.y = parkingspace.space_vcs.parkingspace_right_down_point.y;
//     parking.space_vcs.angle1 = std::atan2(
//         parking.space_vcs.parkingspace_left_down_point.y - parking.space_vcs.parkingspace_left_up_point.y,
//         parking.space_vcs.parkingspace_left_down_point.x - parking.space_vcs.parkingspace_left_up_point.x);
//     parking.space_vcs.angle2 = std::atan2(
//         parking.space_vcs.parkingspace_right_down_point.y - parking.space_vcs.parkingspace_right_up_point.y,
//         parking.space_vcs.parkingspace_right_down_point.x - parking.space_vcs.parkingspace_right_up_point.x);
//     //TODO
//     parking.space_vcs.left_up_point_occ_by_obstacle = false ? 1: 0;
//     parking.space_vcs.right_up_point_occ_by_obstacle = false ? 1: 0;
//     LOG_ERROR("[ParkingSpace]: 角点坐标left-up（%.2f, %.2f）, 角点坐标left-down（%.2f, %.2f）, "
//               "角点坐标right-up（%.2f, %.2f）, 角点坐标right-down（%.2f, %.2f）",
//               parking.space_vcs.parkingspace_left_up_point.x, parking.space_vcs.parkingspace_left_up_point.y,
//               parking.space_vcs.parkingspace_left_down_point.x, parkingspace.space_vcs.parkingspace_left_down_point.y,
//               parking.space_vcs.parkingspace_right_up_point.x, parking.space_vcs.parkingspace_right_up_point.y,
//               parking.space_vcs.parkingspace_right_down_point.x, parking.space_vcs.parkingspace_right_down_point.y);
// //    LOG_INFO("[SurroundVision]: handlePS: parkspace::convert vcs->ipm start ...");
// //    std::vector<Eigen::Vector3d> P_V;
// //    // 角点顺序要正确, 右上(x1,y1)->左上(x2,y2)->左下(x3,y3)->右下(x4,y4)
// //    P_V[0][0] = parking.space_vcs.parkingspace_right_up_point.x;
// //    P_V[0][1] = parking.space_vcs.parkingspace_right_up_point.y;
// //    P_V[1][0] = parking.space_vcs.parkingspace_left_up_point.x;
// //    P_V[1][1] = parking.space_vcs.parkingspace_left_up_point.y;
// //    P_V[2][0] = parking.space_vcs.parkingspace_left_down_point.x;
// //    P_V[2][1] = parking.space_vcs.parkingspace_left_down_point.y;
// //    P_V[3][0] = parking.space_vcs.parkingspace_right_down_point.x;
// //    P_V[3][1] = parking.space_vcs.parkingspace_right_down_point.y;
// //    std::vector<cv::Point2f> P_ipm;
// //    parkingspace_tracker_->vcs2ipm(P_V, P_ipm);
// //    // ipm
// //    parking.space_ipm.parkingspace_right_up_point.x = P_ipm[0].x;
// //    parking.space_ipm.parkingspace_right_up_point.y = P_ipm[0].y;
// //    parking.space_ipm.parkingspace_left_up_point.x = P_ipm[1].x;
// //    parking.space_ipm.parkingspace_left_up_point.y = P_ipm[1].y;
// //    parking.space_ipm.parkingspace_left_down_point.x = P_ipm[2].x;
// //    parking.space_ipm.parkingspace_left_down_point.y = P_ipm[2].y;
// //    parking.space_ipm.parkingspace_right_down_point.x = P_ipm[3].x;
// //    parking.space_ipm.parkingspace_right_down_point.y = P_ipm[3].y;
// //    parking.space_ipm.angle1 = std::atan2(
// //        parking.space_ipm.parkingspace_left_down_point.y - parking.space_ipm.parkingspace_left_up_point.y,
// //        parking.space_ipm.parkingspace_left_down_point.x - parking.space_ipm.parkingspace_left_up_point.x);
// //    parking.space_ipm.angle2 = std::atan2(
// //        parking.space_ipm.parkingspace_right_down_point.y - parking.space_ipm.parkingspace_right_up_point.y,
// //        parking.space_ipm.parkingspace_right_down_point.x - parking.space_ipm.parkingspace_right_up_point.x);
// //    //TODO
// //    parking.space_ipm.left_up_point_occ_by_obstacle = false ? 1: 0;
// //    parking.space_ipm.right_up_point_occ_by_obstacle = false ? 1: 0;
// //    LOG_INFO("[SurroundVision]: handlePS: parkspace::convert vcs->ipm end ...");
//     ps_size++;
//     msg->park_spaces.parking_space[i] = parking;
//   }
//   msg->park_spaces.parking_space_size = output->parkingspace_result.parking_spaces_size;
//   LOG_INFO("[SurroundVision]: 检测到的车位数量：%d", msg->park_spaces.parking_space_size);

}
void SurroundVisionDetector::convert2CaicMsgs(const std::shared_ptr<sim_perception::PerceptionSurround> &output,
                                              std::shared_ptr<caic_perception::PerceptionSurround> &msg_caic) {
  // parking space
//  int ps_size;
//  for (size_t i = 0; i < output->parkingspace_result.parking_spaces_size; i++) {
//    caic_perception::ParkingSpace parking;
//    parking.track_id = output->parkingspace_result.parking_spaces[i].track_id;
//
//    auto parkingspace = output->parkingspace_result.parking_spaces[i];
//    parking.type = parkingspace.type;
//    parking.occu = parkingspace.occu;
//
//    // corner list
//    // vcs
//    parking.space_vcs.parkingspace_left_up_point.x = parkingspace.space_vcs.parkingspace_left_up_point.x;
//    parking.space_vcs.parkingspace_left_up_point.y = parkingspace.space_vcs.parkingspace_left_up_point.y;
//    parking.space_vcs.parkingspace_left_down_point.x = parkingspace.space_vcs.parkingspace_left_down_point.x;
//    parking.space_vcs.parkingspace_left_down_point.y = parkingspace.space_vcs.parkingspace_left_down_point.y;
//    parking.space_vcs.parkingspace_right_up_point.x = parkingspace.space_vcs.parkingspace_right_up_point.x;
//    parking.space_vcs.parkingspace_right_up_point.y = parkingspace.space_vcs.parkingspace_right_up_point.y;
//    parking.space_vcs.parkingspace_right_down_point.x = parkingspace.space_vcs.parkingspace_right_down_point.x;
//    parking.space_vcs.parkingspace_right_down_point.y = parkingspace.space_vcs.parkingspace_right_down_point.y;
//    parking.space_vcs.angle_left = std::atan2(
//        parking.space_vcs.parkingspace_left_down_point.y - parking.space_vcs.parkingspace_left_up_point.y,
//        parking.space_vcs.parkingspace_left_down_point.x - parking.space_vcs.parkingspace_left_up_point.x);
//    parking.space_vcs.angle_right = std::atan2(
//        parking.space_vcs.parkingspace_right_down_point.y - parking.space_vcs.parkingspace_right_up_point.y,
//        parking.space_vcs.parkingspace_right_down_point.x - parking.space_vcs.parkingspace_right_up_point.x);
//    //TODO
//    parking.space_vcs.left_up_point_occ_by_obstacle = false ? 1: 0;
//    parking.space_vcs.right_up_point_occ_by_obstacle = false ? 1: 0;
//
//    LOG_INFO("[SurroundVision]: handlePS: parkspace::convert vcs->ipm start ...");
//    std::vector<Eigen::Vector3d> P_V;
//    // 角点顺序要正确, 右上(x1,y1)->左上(x2,y2)->左下(x3,y3)->右下(x4,y4)
//    P_V[0][0] = parking.space_vcs.parkingspace_right_up_point.x;
//    P_V[0][1] = parking.space_vcs.parkingspace_right_up_point.y;
//    P_V[1][0] = parking.space_vcs.parkingspace_left_up_point.x;
//    P_V[1][1] = parking.space_vcs.parkingspace_left_up_point.y;
//    P_V[2][0] = parking.space_vcs.parkingspace_left_down_point.x;
//    P_V[2][1] = parking.space_vcs.parkingspace_left_down_point.y;
//    P_V[3][0] = parking.space_vcs.parkingspace_right_down_point.x;
//    P_V[3][1] = parking.space_vcs.parkingspace_right_down_point.y;
//    std::vector<cv::Point2f> P_ipm;
//    parkingspace_tracker_->vcs2ipm(P_V, P_ipm);
//    // ipm
//    parking.space_ipm.parkingspace_right_up_point.x = P_ipm[0].x;
//    parking.space_ipm.parkingspace_right_up_point.y = P_ipm[0].y;
//    parking.space_ipm.parkingspace_left_up_point.x = P_ipm[1].x;
//    parking.space_ipm.parkingspace_left_up_point.y = P_ipm[1].y;
//    parking.space_ipm.parkingspace_left_down_point.x = P_ipm[2].x;
//    parking.space_ipm.parkingspace_left_down_point.y = P_ipm[2].y;
//    parking.space_ipm.parkingspace_right_down_point.x = P_ipm[3].x;
//    parking.space_ipm.parkingspace_right_down_point.y = P_ipm[3].y;
//    parking.space_ipm.angle_left = std::atan2(
//        parking.space_ipm.parkingspace_left_down_point.y - parking.space_ipm.parkingspace_left_up_point.y,
//        parking.space_ipm.parkingspace_left_down_point.x - parking.space_ipm.parkingspace_left_up_point.x);
//    parking.space_ipm.angle_right = std::atan2(
//        parking.space_ipm.parkingspace_right_down_point.y - parking.space_ipm.parkingspace_right_up_point.y,
//        parking.space_ipm.parkingspace_right_down_point.x - parking.space_ipm.parkingspace_right_up_point.x);
//    //TODO
//    parking.space_ipm.left_up_point_occ_by_obstacle = false ? 1: 0;
//    parking.space_ipm.right_up_point_occ_by_obstacle = false ? 1: 0;
//    LOG_INFO("[SurroundVision]: handlePS: parkspace::convert vcs->ipm end ...");
//
//    ps_size++;
//    msg_caic->parking_space_result.parking_spaces[i] = parking;
//
//  }
//  msg_caic->parking_space_result.parking_spaces_size = ps_size;
//  LOG_INFO("[SurroundVision]: 检测到的车位数量：", ps_size);
  // freespace
//  freespace_->Postprocess(freespace_result_);
  int x_diff = parkingspace_tracker_->get_ipm2vcs_xdiff();
  int y_diff = parkingspace_tracker_->get_ipm2vcs_ydiff();
  float ratio = parkingspace_tracker_->get_ipm2vcs_ratio();
  int fs_point_size = 0;
  for (size_t i = 0; i < 360; i++) {
    caic_perception::FreespacePoint freespace_point;
    // 第一个点为俯视图9点方向，顺时针取360个点
    float x = -6.24 * std::cos(i * M_PI / 180);
    float y = 6.24 * std::sin(i * M_PI / 180);
    freespace_point.type = 0; // background
    freespace_point.pt.ipm_point.x = x;
    freespace_point.pt.ipm_point.y = y;
    freespace_point.pt.vcs_point.x = (-freespace_point.pt.ipm_point.y + y_diff) / ratio;
    freespace_point.pt.vcs_point.y = (-freespace_point.pt.ipm_point.x + x_diff) / ratio;
    fs_point_size++;
    msg_caic->freespace_result.free_space_points[i] = freespace_point;
    LOG_ERROR("[SurroundVision]: 检测到的可行驶域点集合：%f", freespace_point);
  }
  msg_caic->freespace_result.free_space_points_size = fs_point_size;
  LOG_INFO("[SurroundVision]: 检测到的可行驶域点数量：%d", fs_point_size);
  //ipm2vcs
  msg_caic->info_ipm_vcs.ratio = ratio;
  msg_caic->info_ipm_vcs.x_diff = x_diff;
  msg_caic->info_ipm_vcs.y_diff = y_diff;
}
void SurroundVisionDetector::run() {
  // load config
  if (!Init())
    LOG_ERROR("SurroundVision Init failed!!!");
  std::string loc_topic = "/smartcar/localization/pose";
  std::string planning_topic = "/smartcar/planning_state";
  std::string sur_vision_topic = "/stoic/surround/parkspace";
  static stoic::cm::Subscriber sub_pose = nh_ptr_->subscribe<::caic_localization::LocalizationEstimation, true>(
      loc_topic, 1, std::bind(&SurroundVisionDetector::poseStateCallback, this, std::placeholders::_1));
  static stoic::cm::Subscriber sub_plan = nh_ptr_->subscribe<::caic_interaction::PlanningState, true>(
      planning_topic, 1, std::bind(&SurroundVisionDetector::planningStateCallback, this, std::placeholders::_1));
  static stoic::cm::Subscriber sub_surround = nh_ptr_->subscribe<::sim_perception::PerceptionSurround, true>(
      sur_vision_topic, 1, std::bind(&SurroundVisionDetector::surroundVisionCallback, this, std::placeholders::_1));
  static ::stoic::cm::Publisher pub_sur =
      nh_ptr_->advertise<caic_world::BigFusion, true>("/bfus_pub", 1);
  std::shared_ptr<caic_perception::PerceptionSurround> msg_surround =
      std::make_shared<caic_perception::PerceptionSurround>();  // CAIC 接口
  Rate rate(20);
  while (cm::ok()) {
    if (!is_pos_ready || !is_plan_ready) {
      LOG_ERROR("[SurroundVision] interface is not ready!!!");
    }
    else {
//      convert2CaicMsgs(perc_surround_, msg_surround);
//      pub_sur.publish<caic_perception::PerceptionSurround, true>(*msg_surround);
      // freespace
      std::shared_ptr<caic_world::BigFusion> big_fusion =
          std::make_shared<caic_world::BigFusion>();
      convert2CaicMsgs(perc_surround_, big_fusion);
      pub_sur.publish<caic_world::BigFusion, true>(*big_fusion);


    }
    rate.sleep();
  }
}
bool SurroundVisionDetector::Init() {
  LOG_INFO("[SurroundVision] Init, load vision cfg ...");
  // load pub topic
  std::string topic_cfg = std::string(getInstallRootPath()) + "/config/vision/vision_topics.yaml";
  YAML::Node topic_node;
  try {
    topic_node = YAML::LoadFile(topic_cfg);
  } catch (const YAML::BadFile& e) {
    LOG_ERROR("SurroundVisionDetector load conf yaml error, %s", topic_cfg.c_str());
    return false;
    exit(-1);
  }
  auto pub_topic = topic_node["pub"];
  loadNode<std::string>(pub_topic, sur_pub_str_, "surround_vision");
  loadNode<std::string>(pub_topic, sur_parkingspace_sv_pub_str_, "surround_sv_parking");
  loadNode<std::string>(pub_topic, sur_freespace_sv_pub_str_, "surround_sv_freespace");
  loadNode<std::string>(pub_topic, sur_ipm_mask_sv_pub_str_, "surround_sv_ipm_mask");
  loadNode<std::string>(pub_topic, sur_origin_img_sv_pub_str_, "surround_sv_ipm_origin");
  loadNode<std::string>(pub_topic, bev_pub_str_, "surround_sv_bev");
  LOG_INFO("load surround vision publish topic:  ");
  LOG_INFO("   surround vision: %s", sur_pub_str_.c_str());
  LOG_INFO("   surround bev sv: %s", bev_pub_str_.c_str());
  LOG_INFO("   surround parkingspace sv: %s", sur_parkingspace_sv_pub_str_.c_str());
  LOG_INFO("   surround ipm_mask sv: %s", sur_ipm_mask_sv_pub_str_.c_str());
  LOG_INFO("   surround freespace sv: %s", sur_freespace_sv_pub_str_.c_str());
  LOG_INFO("   surround origin_img sv: %s", sur_origin_img_sv_pub_str_.c_str());
  //parkingspace && freespace process
  std::string ipm_cfg = std::string(getInstallRootPath()) + "/config/vision/ipm.yaml";
  std::string sensor_cfg = getCarIdPath() + "/sensors.cfg";
  LOG_INFO("[SurroundVision] carid path = %s", getCarIdPath().c_str());
  std::string sur_cfg = std::string(getInstallRootPath()) + "/config/vision/surround_cfg.yaml";
  LOG_INFO("[SurroundVision] Init, load ipm cfg = %s", ipm_cfg.c_str());
  LOG_INFO("[SurroundVision] Init, load surround cfg = %s", sur_cfg.c_str());
  parkingspace_tracker_ = std::make_unique<ParkingSpaceTracker>(ipm_cfg, sensor_cfg);
  freespace_ = std::make_unique<SurroundVisionFreespace>(sur_cfg);
  //load hdmap
  YAML::Node hdmap_cfg_node = YAML::LoadFile(sur_cfg);
  hdmap_file = hdmap_cfg_node["hdmap_dir"].as<std::string>();
  if(!hdmap_file.empty())
  {
    LOG_INFO("HDMap load:%s", hdmap_file.data());
    hdmap_flag = apollo::hdmap::HDMapUtil::ReloadMaps(hdmap_file);
    if (!hdmap_flag) {
      LOG_ERROR("HDMap load failed!");
      return false;
    }
  }
  return true;
}
}
