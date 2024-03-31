
//
// Created by xucong on 24-1-3.
//
#include "app/core/sim_perception/front_vision/front_vision_detector.h"
#include "third_party/apollo/math/euler_angles_zxy.h"

using namespace stoic;
using namespace stoic::app::core;

namespace stoic::app::core {
static const char* LOG_TAG =  "front_vision";
void FrontVisionDetector::objCallback(const std::shared_ptr<sim_perception::PerceptionObjects> &msg) {
  perception_objects_ = *msg;
  is_perc_ready = true;
  LOG_INFO("[FrontVisionDetector] received topic is: %s, Msg Type is: %s,",
           FLAGS_sim_vision_objects_topic.c_str(),
           reflection<sim_perception::PerceptionObjects>::fullName().c_str());

}
void FrontVisionDetector::tlCallback(const std::shared_ptr<sim_perception::PerceptionTrafficLights> &msg) {
  if (msg->size == 0) {
    LOG_ERROR("[FrontVisionDetector] input traffic light msg info error, skip!");
    return;
  }
  else {
    perception_traffic_lights_ = *msg;
    is_traffic_light_ready = true;
    LOG_INFO("[FrontVisionDetector] received topic is: %s, Msg Type is: %s,",
             FLAGS_sim_traffic_light_topic.c_str(),
             reflection<sim_perception::PerceptionTrafficLights>::fullName().c_str());
  }
}
void FrontVisionDetector::PoseCallback(const std::shared_ptr<caic_localization::LocalizationEstimation> &msg) {
  if ( (!msg->available &&
        msg->pose.available != caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_POSE) ||
       (!msg->available &&
        msg->pose.available != caic_localization::LocalizationEstimation::LOCALIZATION_ESTIMATION_RELATIVE_POSE)) {
    //
    LOG_ERROR("[FrontVisionDetector] input loc msg info error, skip！！！");
  }
  else {
    pose_manager = msg;
    is_loc_ready = true;
    LOG_INFO("[LocalizationAbsPos] pos x/y/z= %.3f %.3f %.3f, heading= %.3f, velocity x/y/z= %.3f %.3f %.3f",
             msg->pose.pose_info.position.x, msg->pose.pose_info.position.y, msg->pose.pose_info.position.z,
             msg->pose.heading,
             msg->pose.linear_velocity_info.linear_velocity.x,
             msg->pose.linear_velocity_info.linear_velocity.y,
             msg->pose.linear_velocity_info.linear_velocity.z);
  }
}

void FrontVisionDetector::detect(sim_perception::PerceptionObjects &sensor_img) {
//  detect_alg_->ALG_run(sensor_img);
}
void FrontVisionDetector::run() {
  static stoic::cm::Subscriber sub_obj = nh_ptr_->subscribe<::sim_perception::PerceptionObjects, true>(
      FLAGS_sim_vision_objects_topic, 1,
      [this](auto && vision_data) { objCallback(std::forward<decltype(vision_data)>(vision_data)); });
  static stoic::cm::Subscriber sub_tl = nh_ptr_->subscribe<::sim_perception::PerceptionTrafficLights, true>(
      FLAGS_sim_traffic_light_topic, 1,
      [this](auto && tl_data) { tlCallback(std::forward<decltype(tl_data)>(tl_data)); });
  static stoic::cm::Subscriber sub_loc = nh_ptr_->subscribe<::caic_localization::LocalizationEstimation, true>(
      FLAGS_localization_topic, 5,
      [this](auto && loc_data) { PoseCallback(std::forward<decltype(loc_data)>(loc_data)); });
  std::shared_ptr<caic_perception::PerceptionObjects> msg_caic =
      std::make_shared<caic_perception::PerceptionObjects>();  // CAIC 接口
  Rate rate(20);
  while (cm::ok()) {
    if (!is_perc_ready || !is_loc_ready) {
      LOG_ERROR("[FrontVisionDetector] interface is not ready!!!");
    }
    else {
      ConvertObjsToMsg(perception_objects_, msg_caic);
      std::string vision_topic_name = "/vis_sv_pub";
      static ::stoic::cm::Publisher pub_vis_sv =
          nh_ptr_->advertise<caic_perception::PerceptionObjects, true>(vision_topic_name, 1);
      pub_vis_sv.publish<caic_perception::PerceptionObjects, true>(*msg_caic);
//      ObjectsPublish();
      TrafficlightsPublish();
    }
    rate.sleep();
  }
}
void FrontVisionDetector::ObjectsPublish() {
  caic_perception::PerceptionObjects output;
  if (detect_alg_->getDetectOutput(output)) {
    Timer timer;
    output.meta.start_timestamp_us = timer.now();
//    output.meta.sensor_timestamp_us = output.header.stamp;
    for (int i = 0; i < output.objects_size; i++) {
      auto out_obj = output.objects[i];
      LOG_INFO("front_vision",
               "Time_stamp_msec: %f, " /* Time_stamp_msec */
               "Object_id: %d, " /* Object_id, #1 */
               "Object_type: %d, " /* Object_type, #7 */
               "Object_L: %f, " /* Object_L, #9 */
               "Object_W: %f, " /* Object_W, #10 */
               "Object_H: %f, " /* Object_H, #11 */
               "Object_vcs_vx: %f, " /* Object_vcs_vx, #12 */
               "Object_vcs_vy: %f, " /* Object_vcs_vy, #13 */
               "Object_vcs_px: %f, " /* Object_vcs_px, #14 */
               "Object_vcs_py: %f, " /* Object_vcs_py, #15 */
               "Object_vcs_theta: %f, " /* Object_vcs_theta, #16 */ ,
               output.header.stamp / 1e6,
               out_obj.id,
               out_obj.type,
               out_obj.length,
               out_obj.width,
               out_obj.height,
               out_obj.pose_in_vcs.velocity.x,
               out_obj.pose_in_vcs.velocity.y,
               out_obj.pose_in_vcs.position.x,
               out_obj.pose_in_vcs.position.y,
               out_obj.pose_in_vcs.heading
      );
      LOG_INFO("front_vision", "\n");
    }
    std::string vision_topic_name = "/vis_sv_pub";
    static ::stoic::cm::Publisher pub_vis_sv =
        nh_ptr_->advertise<caic_perception::PerceptionObjects, true>(vision_topic_name, 1);
    pub_vis_sv.publish<caic_perception::PerceptionObjects, true>(output);
  }
}
void FrontVisionDetector::TrafficlightsPublish() {
  std::string tl_topic_name = "/ad/vision/tl/trafficlight";
  //output to caic
  std::shared_ptr<caic_perception::PerceptionTrafficLights> msg_caic =
      std::make_shared<caic_perception::PerceptionTrafficLights>();
  ConvertTl2CAICMsg(perception_traffic_lights_, msg_caic);
  // To Inter-Process, format: caic_perception::PerceptionTrafficLights
  static ::stoic::cm::Publisher pub_vis =
      nh_ptr_->advertise<caic_perception::PerceptionTrafficLights, true>(tl_topic_name, 1);
  pub_vis.publish<caic_perception::PerceptionTrafficLights, true>(*msg_caic);
}
void FrontVisionDetector::ConvertObjsToMsg(const sim_perception::PerceptionObjects &output,
                                           std::shared_ptr<caic_perception::PerceptionObjects> &perc_objs) {
  Timer timer;
  static int seq = 0;
  perc_objs->header.stamp = output.header.stamp;
  perc_objs->header.seq = output.header.seq;
  perc_objs->meta.start_timestamp_us = timer.now();
  perc_objs->meta.sensor_timestamp_us = output.header.stamp;
  LOG_INFO("[FrontVisionDetector] 接收的序列号: %u,时间戳: %ld",output.header.seq,output.header.stamp);
  // vcs to map
  manif::SE3d TX_V_M;
  Eigen::Quaterniond q(pose_manager->pose.pose_info.quaternion.w, pose_manager->pose.pose_info.quaternion.x,
                       pose_manager->pose.pose_info.quaternion.y, pose_manager->pose.pose_info.quaternion.z);
  Eigen::Vector3d t(pose_manager->pose.pose_info.position.x,
                    pose_manager->pose.pose_info.position.y,
                    pose_manager->pose.pose_info.position.z);
  manif::SE3d pose(manif::SE3d(t, q));
  getV2MTransform(pose, TX_V_M);
  Eigen::Matrix3d TX_V_M_rotation = TX_V_M.rotation();
  float yaw = std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0));
  perc_objs->objects_size = 0;
  int objects_size = 0;
  for (size_t i = 0; i < output.objects_size; i++) {
    sim_perception::PerceptionObject msg_obj = output.objects[i];
    caic_perception::PerceptionObject caic_obj;
    if (msg_obj.id == 0) {
      LOG_ERROR("感知障碍物ID错误！！！");
      continue;
    }
    caic_obj.id = msg_obj.id + 1000;
    LOG_ERROR("感知障碍物ID：%d",  caic_obj.id);
    switch ((sim_perception::ObstacleType)msg_obj.type) {
      case sim_perception::ObstacleType::CAR:
        caic_obj.type = caic_perception::ObstacleType::VEHICLE;
        caic_obj.veh_sub_type = caic_perception::VehicleSubType::CAR;
        break;
      case sim_perception::ObstacleType::TRUCK:
        caic_obj.type = caic_perception::ObstacleType::VEHICLE;
        caic_obj.veh_sub_type = caic_perception::VehicleSubType::TRUCK;
        break;
      case sim_perception::ObstacleType::VAN:
        caic_obj.type = caic_perception::ObstacleType::VEHICLE;
        caic_obj.veh_sub_type = caic_perception::VehicleSubType::VAN;
        break;
      case sim_perception::ObstacleType::BUS:
        caic_obj.type = caic_perception::ObstacleType::VEHICLE;
        caic_obj.veh_sub_type = caic_perception::VehicleSubType::BUS;
        break;
      case sim_perception::ObstacleType::BIKE:
        caic_obj.type = caic_perception::ObstacleType::CYCLIST;
        caic_obj.cyclist_sub_type = caic_perception::CyclistSubType::BICYCLE;
        break;
      case sim_perception::ObstacleType::MOTORBIKE:
        caic_obj.type = caic_perception::ObstacleType::CYCLIST;
        caic_obj.cyclist_sub_type = caic_perception::CyclistSubType::MOTORCYCLE;
        break;
      case sim_perception::ObstacleType::PEDESTRIAN:
        caic_obj.type = caic_perception::ObstacleType::PEDESTRIAN;
        break;
      case sim_perception::ObstacleType::TRAFFIC_SIGN:
        caic_obj.type = caic_perception::ObstacleType::TRAFFIC_SIGN;
        break;
      case sim_perception::ObstacleType::BARRIER:
        caic_obj.type = caic_perception::ObstacleType::TRAFFIC_BARRIER;
        break;
      default:
        caic_obj.type = caic_perception::ObstacleType::UNKNOWN;
        break;
    }
    caic_obj.valid = true;
#if defined MW_ROS_IPC || defined MW_ROS_ORIN
    LOG_INFO("Using ros platform！！！");
    caic_obj.length = msg_obj.length;
    caic_obj.width = msg_obj.width;
    caic_obj.height = msg_obj.height;
    caic_obj.pose_in_world.position.x = msg_obj.pose_in_world.position.x;
    caic_obj.pose_in_world.position.y = msg_obj.pose_in_world.position.y;
    caic_obj.pose_in_world.position.z = msg_obj.pose_in_world.position.z;
    caic_obj.pose_in_world.velocity.x = msg_obj.pose_in_world.velocity.x;
    caic_obj.pose_in_world.velocity.y = msg_obj.pose_in_world.velocity.y;
    caic_obj.pose_in_world.velocity.z = msg_obj.pose_in_world.velocity.z;
    caic_obj.pose_in_world.acceleration.x = msg_obj.pose_in_world.acceleration.x;
    caic_obj.pose_in_world.acceleration.y = msg_obj.pose_in_world.acceleration.y;
    caic_obj.pose_in_world.acceleration.z = msg_obj.pose_in_world.acceleration.z;
    caic_obj.pose_in_world.heading = msg_obj.pose_in_world.heading;
#elif defined MW_ART_IPC || defined MW_ART_ORIN
    LOG_INFO("Using Apollo CyberRT platform！！！");
    // map
    Eigen::Vector3d vcs_pos(msg_obj.pose_in_vcs.position.x,
                            msg_obj.pose_in_vcs.position.y,
                            msg_obj.pose_in_vcs.position.z);
    Eigen::Vector3d map_pos = TX_V_M.act(vcs_pos);
    caic_obj.pose_in_world.position.x = map_pos.x();
    caic_obj.pose_in_world.position.y = map_pos.y();
    caic_obj.pose_in_world.position.z = map_pos.z();
    Eigen::Vector3d vcs_vel(msg_obj.pose_in_vcs.velocity.x,
                            msg_obj.pose_in_vcs.velocity.y,
                            msg_obj.pose_in_vcs.velocity.z);
    Eigen::Vector3d map_vel = TX_V_M_rotation * vcs_vel;
    if (std::hypot(map_vel.x(), map_vel.y()) < 1.0) {
      caic_obj.pose_in_world.velocity.x = 0.0;
      caic_obj.pose_in_world.velocity.y = 0.0;
    } else {
      caic_obj.pose_in_world.velocity.x = map_vel.x();
      caic_obj.pose_in_world.velocity.y = map_vel.y();
    }
    caic_obj.pose_in_world.heading = msg_obj.pose_in_vcs.heading + yaw + M_PI_2;
    caic_obj.pose_in_world.position_reliable = 1;
    caic_obj.pose_in_world.velocity_reliable = 1;
    caic_obj.pose_in_world.acceleration_reliable = 0;
    // vcs
    caic_obj.pose_in_vcs.position.x = msg_obj.pose_in_vcs.position.x;
    caic_obj.pose_in_vcs.position.y = msg_obj.pose_in_vcs.position.y;
    caic_obj.pose_in_vcs.position.z = msg_obj.pose_in_vcs.position.z;
    if (std::hypot(msg_obj.pose_in_vcs.position.x, msg_obj.pose_in_vcs.position.y) < 1.0) {
      caic_obj.pose_in_vcs.velocity.x = 0.0;
      caic_obj.pose_in_vcs.velocity.y = 0.0;
    } else {
      caic_obj.pose_in_vcs.velocity.x = msg_obj.pose_in_vcs.velocity.x;
      caic_obj.pose_in_vcs.velocity.y = msg_obj.pose_in_vcs.velocity.y;
    }
    caic_obj.pose_in_vcs.heading = msg_obj.pose_in_vcs.heading;
    caic_obj.pose_in_vcs.position_reliable = 1;
    caic_obj.pose_in_vcs.velocity_reliable = 1;
    caic_obj.pose_in_vcs.acceleration_reliable = 0;
    caic_obj.length = msg_obj.length;
    caic_obj.width  = msg_obj.width;
    caic_obj.height = msg_obj.height;
    // set available
    caic_obj.avaliable = 0;
    caic_obj.avaliable |= caic_obj.PERCEPTION_OBJECT_TYPE;
    caic_obj.avaliable |= caic_obj.PERCEPTION_OBJECT_VALID;
    caic_obj.avaliable |= caic_obj.PERCEPTION_OBJECT_POSE_IN_WORLD;
    caic_obj.avaliable |= caic_obj.PERCEPTION_OBJECT_POSE_IN_VCS;
    caic_obj.avaliable |= caic_obj.PERCEPTION_OBJECT_LENGTH;
    caic_obj.avaliable |= caic_obj.PERCEPTION_OBJECT_WIDTH;
    caic_obj.avaliable |= caic_obj.PERCEPTION_OBJECT_HEIGHT;
    caic_obj.avaliable |= caic_obj.PERCEPTION_OBJECT_COVARIANCE;
#endif
    perc_objs->objects[objects_size] = caic_obj;
    objects_size++;
    if(objects_size > OBJ_SIZE) {
      LOG_ERROR("[FrontVisionDetector] vehicle objects size > OBJ_SIZE");
      exit(-1);
    }
  }
  perc_objs->objects_size = objects_size;
  LOG_INFO("感知障碍物数量：%d\n",  perc_objs->objects_size);
}
void FrontVisionDetector::ConvertTl2CAICMsg(const sim_perception::PerceptionTrafficLights &output,
                                            std::shared_ptr<caic_perception::PerceptionTrafficLights> &msg_caic) {
  msg_caic->header.stamp = Timer::now();
  msg_caic->meta.start_timestamp_us = Timer::now();
  msg_caic->meta.sensor_timestamp_us = output.header.stamp;
  msg_caic->size = 0;
  int tl_size = 0;
  for(size_t i = 0; i < output.size; i++){
    caic_perception::TrafficLight traffic_light;
    traffic_light.avaliable = 1;
    traffic_light.color = tl_color_[output.traffic_lights[i].color];
    traffic_light.show_type = tl_show_type_[output.traffic_lights[i].show_type];
    traffic_light.id[0] = '0';
    traffic_light.confidence = 1.0;
    traffic_light.blink = false;
    traffic_light.remaining_time = output.traffic_lights[i].remaining_time;
    msg_caic->traffic_lights[i] = traffic_light;
    tl_size++;
    if(tl_size > TLIGHT_INFO_SIZE) {
      LOG_ERROR("[FrontVisionDetector] traffic light size > TLIGHT_INFO_SIZE");
      exit(-1);
    }
    LOG_ERROR("红绿灯颜色：%d, 倒计时：%ds", traffic_light.color, traffic_light.remaining_time);
  }
  msg_caic->size = tl_size;
  LOG_INFO("感知红绿灯数量：%d\n",  msg_caic->size);
}
} //namespace stoic::app::core
