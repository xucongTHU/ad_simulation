
// Copyright 2022 The XUCONG Authors. All Rights Reserved.
#include "app/bridge/sensor/object_sensor.h"
#include "common/adapters/adapter_gflags.h"
#include "common/log/Logger.h"
#include "common/config.h"

using namespace stoic;
using namespace stoic::app::bridge;
static const char* LOG_TAG = "main";
namespace stoic::app::bridge {
void ObjectSensor::run() {
  sim_ground_truth::FrontVisionObjects objects{};
  sim_perception::PerceptionObjects msg_objs{};
  std::string config_file = std::string(getInstallRootPath()) + "/config/bridge/config.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file);
  init(config_node);
  // connect to client
  connectToUdpClient(bind_port_);
  //
  static ::stoic::cm::Publisher pub_vis =
    nh_ptr_->advertise<sim_perception::PerceptionObjects, true>(FLAGS_sim_vision_objects_topic, 1);
  Rate rate(50);
  while (stoic::cm::ok()) {
    recvData(objects);
    convert2ObjMsg(objects, msg_objs);
    pub_vis.publish<sim_perception::PerceptionObjects, true>(msg_objs);
    rate.sleep();
  }
}
bool ObjectSensor::convert2ObjMsg(const sim_ground_truth::FrontVisionObjects &front_vision_objects,
                                        sim_perception::PerceptionObjects &msg) {
  // header
  msg.header.stamp = Timer::now();
  msg.header.seq = seq_num_++;
  int objects_size = 0;
  uint32_t noElements = sizeof(front_vision_objects.object) / sizeof(sim_ground_truth::ObjectData);
  msg.objects_size = noElements;
  LOG_INFO("====ProcVisionInfo: got start of frame===");
  LOG_INFO("simTime = %.3lf, simFrame = %ld", front_vision_objects.header.simTime, front_vision_objects.header.frameNo );
  for (uint32_t i = 0; i < noElements; i++) {
    sim_ground_truth::ObjectData object_data = front_vision_objects.object[i];
    if (object_data.id == 0) {
      LOG_ERROR("接收障碍物失败");
      continue;
    }
    msg.objects[i].id = object_data.id;
    switch (object_data.type) {
      case 0:
        msg.objects[i].type = sim_perception::ObstacleType::UNKNOWN;
        break;
      case 1:
        msg.objects[i].type = sim_perception::ObstacleType::CAR;
        break;
      case 2:
        msg.objects[i].type = sim_perception::ObstacleType::TRUCK;
        break;
      case 3:
        msg.objects[i].type = sim_perception::ObstacleType::VAN;
        break;
      case 4:
        msg.objects[i].type = sim_perception::ObstacleType::BIKE;
        break;
      case 5:
        msg.objects[i].type = sim_perception::ObstacleType::PEDESTRIAN;
        break;
      case 9:
        msg.objects[i].type = sim_perception::ObstacleType::BARRIER;
        break;
      case 13:
        msg.objects[i].type = sim_perception::ObstacleType::MOTORBIKE;
        break;
      case 14:
        msg.objects[i].type = sim_perception::ObstacleType::BUS;
        break;
      case 16:
        msg.objects[i].type = sim_perception::ObstacleType::TRAFFIC_SIGN;
        break;
      case 18:
        msg.objects[i].type = sim_perception::ObstacleType::TRAILER;
        break;
      default:break;
    }
    // map coordinate
    msg.objects[i].pose_in_world.position.x = object_data.pos.x;
    msg.objects[i].pose_in_world.position.y = object_data.pos.y;
    msg.objects[i].pose_in_world.position.z = object_data.pos.z;
    msg.objects[i].pose_in_world.heading = object_data.heading;
    msg.objects[i].pose_in_world.velocity.x = object_data.speed.x;
    msg.objects[i].pose_in_world.velocity.y = object_data.speed.y;
    msg.objects[i].pose_in_world.velocity.z = object_data.speed.z;
    msg.objects[i].pose_in_world.acceleration.x = object_data.accel.x;
    msg.objects[i].pose_in_world.acceleration.y = object_data.accel.y;
    msg.objects[i].pose_in_world.acceleration.z = object_data.accel.z;
    // vcs coordinate
    msg.objects[i].pose_in_vcs.position.x = object_data.pos.x;
    msg.objects[i].pose_in_vcs.position.y = object_data.pos.y;
    msg.objects[i].pose_in_vcs.position.z = object_data.pos.z;
    msg.objects[i].pose_in_vcs.heading = object_data.heading;
    msg.objects[i].pose_in_vcs.pitch = object_data.pitch;
    msg.objects[i].pose_in_vcs.roll = object_data.roll;
    msg.objects[i].pose_in_vcs.heading = object_data.heading;
    msg.objects[i].pose_in_vcs.velocity.x = object_data.speed.x;
    msg.objects[i].pose_in_vcs.velocity.y = object_data.speed.y;
    msg.objects[i].pose_in_vcs.velocity.z = object_data.speed.z;
    msg.objects[i].pose_in_vcs.acceleration.x = object_data.accel.x;
    msg.objects[i].pose_in_vcs.acceleration.y = object_data.accel.y;
    msg.objects[i].pose_in_vcs.acceleration.z = object_data.accel.z;
    msg.objects[i].length = object_data.geo.dimX;
    msg.objects[i].width  = object_data.geo.dimY;
    msg.objects[i].height = object_data.geo.dimZ;
    objects_size++;
    LOG_ERROR("handling Front Vision [id: %d]", msg.objects[i].id);
  }
  msg.objects_size = objects_size;
  LOG_ERROR("[Vision] 接收障碍物数量：%d", msg.objects_size);
  LOG_INFO("====ProcVisionInfo: got end of frame===\n");
}
bool ObjectSensor::init(const YAML::Node &node) {
  std::string key = "obstacle";
  YAML::Node result = common::YamlUtil::GetValue(node, key);
  if (!result.IsNull() && result.IsMap()) {
    bind_port_ = result["bind_port"].as<int>();
  } else {
    LOG_ERROR("Key [%s] is not found in the YAML data", key.c_str());
  }
}
}  // namespace stoic::app::bridge
