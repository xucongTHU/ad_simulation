
// Copyright 2022 The XUCONG Authors. All Rights Reserved.
#include "app/bridge/sensor/lane_line_sensor.h"
#include "common/adapters/adapter_gflags.h"
#include "common/log/Logger.h"

using namespace stoic;
using namespace stoic::app::bridge;
static const char* LOG_TAG = "main";
namespace stoic::app::bridge {
void LaneLineSensor::run() {
  sim_ground_truth::LaneLines lane_lines;
  sim_perception::PerceptionLanes msg_lanes;
  //
  static ::stoic::cm::Publisher pub_perc =
    nh_ptr_->advertise<sim_perception::PerceptionLanes, true>(FLAGS_sim_vision_lane_topic, 1);
  Rate rate(50);
  while (stoic::cm::ok()) {
    recvData(lane_lines);
    convert2LanesMsg(lane_lines, msg_lanes);
    pub_perc.publish<sim_perception::PerceptionLanes, true>(msg_lanes);
    rate.sleep();
  }
}
bool LaneLineSensor::convert2LanesMsg(const sim_ground_truth::LaneLines &lane_lines, sim_perception::PerceptionLanes &msg) {
  // header
  msg.header.stamp = Timer::now();
  msg.header.seq = ++seq_num_;
  int lanes_size = 0;
  uint32_t noElements = sizeof(lane_lines.lane) / sizeof(sim_ground_truth::LaneLine);
  msg.size = noElements;
  LOG_INFO("=======ProcLaneInfo: got start of frame=======");
  LOG_INFO("  simTime = %.3lf, simFrame = %ld", lane_lines.header.simTime, lane_lines.header.frameNo );
  for (uint32_t i = 0; i < noElements; i++) {    // msg header
    sim_ground_truth::LaneLine lane_line = lane_lines.lane[i];
    msg.lanes[i].id = lane_line.laneId;
    switch (lane_line.color) {
      case ROADMARK_COLOR_WHITE:
        msg.lanes[i].color = sim_perception::LaneColor::WHITE;
        break;
      case ROADMARK_COLOR_YELLOW:
        msg.lanes[i].color = sim_perception::LaneColor::YELLOW;
        break;
      default:
        msg.lanes[i].color = sim_perception::LaneColor::UNKNOWN;
        break;
    }
    switch (lane_line.type) {
      case ROADMARK_TYPE_SOLID:
        msg.lanes[i].type = sim_perception::LaneType::SOLID;
        LOG_INFO("当前车道线是实线， 第%d条车道线！！！", i+1);
        break;
      case ROADMARK_TYPE_BROKEN:
        msg.lanes[i].type = sim_perception::LaneType::DASHED;
        LOG_INFO("当前车道线是虚线， 第%d条车道线！！！", i+1);
        break;
      case ROADMARK_TYPE_SOLID_SOLID:
        msg.lanes[i].type = sim_perception::LaneType::DOUBLE_SOLID;
        LOG_INFO("当前车道线是双实线， 第%d条车道线！！！", i+1);
        break;
      default:
        msg.lanes[i].type = sim_perception::LaneType::UNKNOWN;
        LOG_INFO("当前车道线是未知， 第%d条车道线！！！", i+1);
        break;
    }
    if (lane_line.type == ROADMARK_TYPE_CURB) {
      msg.lanes[i].type = sim_perception::LaneType::ROAD_EDGE;
    }
    // 判断lane index, 根据本车LaneID判断
    LOG_INFO("本车当前车道ID= %d", lane_lines.lane[i].curLaneId);
    LOG_INFO("传感器识别当前车道线ID= %d", lane_lines.lane[i].laneId);
    if ( lane_line.curLaneId < 0 && lane_line.laneId == lane_line.curLaneId + 1 ) {
      msg.lanes[i].index = sim_perception::LaneIndex::LANE_LKA_LEFT;
      LOG_INFO("当前是左线， 第%d条车道线！！！", i+1);
    } else if ( lane_line.curLaneId < 0 && lane_line.laneId == lane_line.curLaneId) {
      msg.lanes[i].index = sim_perception::LaneIndex::LANE_LKA_RIGHT;
      LOG_INFO("当前是右线， 第%d条车道线！！！", i+1);
    } else if ( lane_line.curLaneId < 0 && lane_line.laneId ==  lane_line.curLaneId + 2) {
      msg.lanes[i].index = sim_perception::LaneIndex::LANE_NEXT_LEFT;
      LOG_INFO("当前是左左线， 第%d条车道线！！！", i+1);
    } else if ( lane_line.curLaneId < 0 && lane_line.laneId == lane_line.curLaneId - 1 ) {
      msg.lanes[i].index = sim_perception::LaneIndex::LANE_NEXT_RIGHT;
      LOG_INFO("当前是右右线， 第%d条车道线！！！", i + 1);
    } else if ( lane_line.curLaneId > 0 && lane_line.laneId == lane_line.curLaneId - 1 ) {
      msg.lanes[i].index = sim_perception::LaneIndex::LANE_LKA_LEFT;
      LOG_INFO("当前是左线， 第%d条车道线！！！", i + 1);
    } else if ( lane_line.curLaneId > 0 && lane_line.laneId == lane_line.curLaneId) {
      msg.lanes[i].index = sim_perception::LaneIndex::LANE_LKA_RIGHT;
      LOG_INFO("当前是右线， 第%d条车道线！！！", i + 1);
    } else if ( lane_line.curLaneId > 0 && lane_line.laneId ==  lane_line.curLaneId - 2) {
      msg.lanes[i].index = sim_perception::LaneIndex::LANE_NEXT_LEFT;
      LOG_INFO("当前是左左线， 第%d条车道线！！！", i + 1);
    } else if ( lane_line.curLaneId > 0 && lane_line.laneId == lane_line.curLaneId + 1 ) {
      msg.lanes[i].index = sim_perception::LaneIndex::LANE_NEXT_RIGHT;
      LOG_INFO("当前是右右线， 第%d条车道线！！！", i + 1);
    } else {
      msg.lanes[i].index = sim_perception::LaneIndex::LANE_NOT_ASSIGNED;
      LOG_INFO("车道线错误！！！");
    }
    //车道线拟合
    msg.lanes[i].poly.start_x = lane_line.poly.start_x;
    msg.lanes[i].poly.end_x = lane_line.poly.end_x;
    msg.lanes[i].poly.c0 = lane_line.poly.c0;
    msg.lanes[i].poly.c1 = lane_line.poly.c1;
    msg.lanes[i].poly.c2 = lane_line.poly.c2;
    msg.lanes[i].poly.c3 = lane_line.poly.c3;
    msg.lanes[i].points_size = lane_line.noDataPoints;
    msg.lanes[i].is_predict = false;
    msg.lanes[i].width = 3.5; //默认3.5m
    msg.lanes[i].avaliable = sim_perception::PerceptionLane::PERCEPTION_LANE_ID  |
        sim_perception::PerceptionLane::PERCEPTION_LANE_COLOR |
        sim_perception::PerceptionLane::PERCEPTION_LANE_TYPE  |
        sim_perception::PerceptionLane::PERCEPTION_LANE_INDEX |
        sim_perception::PerceptionLane::PERCEPTION_LANE_SCORE |
        sim_perception::PerceptionLane::PERCEPTION_LANE_POLY  |
        // sim_perception::PerceptionLane::PERCEPTION_LANE_POINTS |
        sim_perception::PerceptionLane::PERCEPTION_LANE_IS_PREDICT  |
        sim_perception::PerceptionLane::PERCEPTION_LANE_WIDTH |
        sim_perception::PerceptionLane::PERCEPTION_LANE_STOP_LINE_TYPE;

    LOG_ERROR("lane start_x= %.3f, end_x=%.3f, lane poly coefficient c0/c1/c2/c3 = %.3f/%.3f/%.3f/%.3f\n",
              msg.lanes[i].poly.start_x, msg.lanes[i].poly.end_x,
              msg.lanes[i].poly.c0, msg.lanes[i].poly.c1, msg.lanes[i].poly.c2, msg.lanes[i].poly.c3);
  }
  LOG_INFO("=======ProcLaneInfo: got end of frame=======\n");
}
bool LaneLineSensor::init(const YAML::Node &node) {
  std::string key = "lane";
  YAML::Node result = common::YamlUtil::GetValue(node, key);
  if (!result.IsNull() && result.IsMap()) {
    bind_port_ = result["bind_port"].as<int>();
  } else {
    LOG_ERROR("Key [%s] is not found in the YAML data", key.c_str());
  }
}
}  // namespace stoic::app::bridge
