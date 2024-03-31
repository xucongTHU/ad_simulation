
//
// Created by xucong on 24-1-3.
//
#include "app/core/sim_perception/front_vision/lane_line.h"

using namespace stoic;
using namespace stoic::app::core;
static const char* LOG_TAG =  "lane";

namespace stoic::app::core {
void LaneLine::PercLanesCallback(const std::shared_ptr<sim_perception::PerceptionLanes> &msg) {
  sim_perc_lanes_ = *msg;
  is_sim_perc_ready = true;
  LOG_INFO("===========perc lane instance===========\n"
           "  received topic is: %s, , Msg Type is: %s,",
           FLAGS_sim_vision_lane_topic.c_str(),
           reflection<sim_perception::PerceptionLanes>::fullName().c_str());
}
void LaneLine::run() {
  static stoic::cm::Subscriber sub_lane = nh_ptr_->subscribe<::sim_perception::PerceptionLanes, true>(
      FLAGS_sim_vision_lane_topic, 1,
      [this](auto && lane_data) { PercLanesCallback(std::forward<decltype(lane_data)>(lane_data)); });
  std::string lane_sv_topic_name = "/lane_line_sv_pub";
  static ::stoic::cm::Publisher perception_pub =
      nh_ptr_->advertise<caic_perception::PerceptionLanes, true>(
          lane_sv_topic_name, 5);
  Rate rate(20);
  while (cm::ok()) {
    std::shared_ptr<caic_perception::PerceptionLanes> msg_caic =
        std::make_shared<caic_perception::PerceptionLanes>();  // CAIC 接口
    if (!is_sim_perc_ready) {
      LOG_ERROR("sim perception lanes not ready!!!");
    }
    else {
      ConvertLaneRes2CAICMsg(sim_perc_lanes_, msg_caic);
      perception_pub.publish<caic_perception::PerceptionLanes, true>(*msg_caic);
    }
    rate.sleep();
  }
}
void LaneLine::ConvertLaneRes2CAICMsg(
    const sim_perception::PerceptionLanes &output,
    std::shared_ptr<caic_perception::PerceptionLanes> &perc_lanes) {
  LOG_INFO("================== [lane line detector] ==================");
  Timer timer;
  static int seq = 0;
  perc_lanes->header.stamp = output.header.stamp;
  perc_lanes->header.seq = output.header.seq;
  perc_lanes->meta.start_timestamp_us = timer.now();
  perc_lanes->meta.sensor_timestamp_us = output.header.stamp;
  perc_lanes->size = output.size;
  LOG_INFO("[perc] 接收的序列号: %u,时间戳: %ld",output.header.seq,output.header.stamp);
  perc_lanes->lanes_size = 0;
  int lanes_size = 0;
  for (size_t i = 0; i < output.size; i++) {
    sim_perception::PerceptionLane msg_lane = output.lanes[i];
    caic_perception::PerceptionLane caic_lane;
    caic_lane.id = msg_lane.id;
    LOG_ERROR("感知车道线ID：%d",  caic_lane.id);
    caic_lane.color = caic_perception::LaneColor::UNKNOWN;
    switch (type_convert_[msg_lane.type]) {
      case 1: {}
        caic_lane.type = caic_perception::LaneType::SOLID;
        break;
      case 3:
        caic_lane.type = caic_perception::LaneType::DASHED;
        break;
      case 7:
        caic_lane.type = caic_perception::LaneType::DOUBLE_SOLID;
        break;
      case 6:
        caic_lane.type = caic_perception::LaneType::DOUBLE_DASHED;
        break;
      default:
        caic_lane.type = caic_perception::LaneType::UNKNOWN;
        break;
    }
    if (type_convert_[msg_lane.type] == 2){
      caic_lane.type = caic_perception::LaneType::ROAD_EDGE;
    }
    if (index_convert_[msg_lane.index] == "L") {
      caic_lane.index = caic_perception::LaneIndex::LANE_LKA_LEFT;
      LOG_INFO("感知车道线是左线！！！");
    } else if (index_convert_[msg_lane.index] == "R") {
      caic_lane.index = caic_perception::LaneIndex::LANE_LKA_RIGHT;
      LOG_INFO("感知车道线是右线！！！");
    } else if (index_convert_[msg_lane.index] == "LL") {
      caic_lane.index = caic_perception::LaneIndex::LANE_NEXT_LEFT;
      LOG_INFO("感知车道线是左左线！！！");
    } else if (index_convert_[msg_lane.index] == "RR") {
      caic_lane.index = caic_perception::LaneIndex::LANE_NEXT_RIGHT;
      LOG_INFO("感知车道线是右右线！！！");
    } else {
      caic_lane.index = caic_perception::LaneIndex::LANE_NOT_ASSIGNED;
    }
    caic_lane.score = 1.0;
    caic_lane.poly.c0 = msg_lane.poly.c0;
    caic_lane.poly.c1 = msg_lane.poly.c1;
    caic_lane.poly.c2 = msg_lane.poly.c2;
    caic_lane.poly.c3 = msg_lane.poly.c3;
    caic_lane.poly.start_x = msg_lane.poly.start_x;
    caic_lane.poly.end_x = msg_lane.poly.end_x;
    caic_lane.points_size = msg_lane.points_size;
    caic_lane.is_predict = msg_lane.is_predict;
    caic_lane.width = msg_lane.width;
    caic_lane.stop_line_type = caic_perception::StopLineType::UNKNOWN;
    caic_lane.avaliable = caic_perception::PerceptionLane::PERCEPTION_LANE_ID
        | caic_perception::PerceptionLane::PERCEPTION_LANE_COLOR
        | caic_perception::PerceptionLane::PERCEPTION_LANE_TYPE
        | caic_perception::PerceptionLane::PERCEPTION_LANE_INDEX
        | caic_perception::PerceptionLane::PERCEPTION_LANE_SCORE
        | caic_perception::PerceptionLane::PERCEPTION_LANE_POLY
            // | caic_perception::PerceptionLane::PERCEPTION_LANE_POINTS
        | caic_perception::PerceptionLane::PERCEPTION_LANE_IS_PREDICT
        | caic_perception::PerceptionLane::PERCEPTION_LANE_WIDTH
        | caic_perception::PerceptionLane::PERCEPTION_LANE_STOP_LINE_TYPE;
    if(lanes_size < PERC_LANES_SIZE){
      perc_lanes->lanes[lanes_size] = caic_lane;
      lanes_size++;
    }
  }
  perc_lanes->lanes_size = lanes_size;
  LOG_INFO("感知车道线数量：%d\n",  perc_lanes->lanes_size);
}
} //namespace stoic::app::core
