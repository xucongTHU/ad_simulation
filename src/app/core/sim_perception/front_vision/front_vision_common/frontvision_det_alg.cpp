
//
// Created by xucong on 24-1-19.
//
#include "frontvision_det_alg.h"

namespace stoic::app::core {
static const char* LOG_TAG =  "front_vision_alg";
bool FrontVisionDetectAlg::ALG_run(sim_perception::PerceptionObjects &msg) {
  LOG_INFO("############# FrontVisionDetectAlg::ALG_run #############");
  caic_perception::PerceptionObjects output;
  static int seq = 0;
  std::memset(&output, 0, sizeof(caic_perception::PerceptionObjects));
  output.header.stamp = Timer::now();
  output.header.seq = seq++;
  ConvertObjsToMsg(msg, output);  // caic msg
  LOG_ERROR("output obj size : %d", output.objects_size);

//    std::unique_lock<std::mutex> ulock(mutex_detect_results_);
//    detect_results_.emplace(output);
    LOG_ERROR("+++++++Step2+++++++++++++");

}
bool FrontVisionDetectAlg::getModelOutput(caic_perception::PerceptionObjects &output) {
  std::unique_lock<std::mutex> ulock(mutex_model_results_);
  if (model_results_.empty()) {
    LOG_ERROR("FrontVisionDetectAlg detect results is empty!");
    return false;
  }
  output = model_results_.front();
  model_results_.pop();
  return true;
}
bool FrontVisionDetectAlg::getDetectOutput(caic_perception::PerceptionObjects &output) {
  LOG_ERROR("+++++++Step3+++++++++++++");
//  std::unique_lock<std::mutex> ulock(mutex_detect_results_);
  if(detect_results_.empty()){
    LOG_ERROR("FrontVisionDetectAlg detect results is empty!");
    return false;
  }
  output = detect_results_.front();
  detect_results_.pop();
  return true;
}
bool FrontVisionDetectAlg::getTrafficLtDetectOutput(DetectResultTls &output) {
  std::unique_lock<std::mutex> ulock(mutex_trafficlght_results_);
  if(trafficlght_results_.empty()){
    LOG_ERROR("FrontVisionDetectAlg detect results is empty!");
    return false;
  }
  output = trafficlght_results_.front();
  trafficlght_results_.pop();
  return true;
}
void FrontVisionDetectAlg::ConvertObjsToMsg(sim_perception::PerceptionObjects &sim,
                                            caic_perception::PerceptionObjects &output) {
  int objects_size = 0;
  for (size_t i = 0; i < sim.objects_size; i++) {
    caic_perception::PerceptionObject obj;
    obj.id = sim.objects[i].id;
    switch ((sim_perception::ObstacleType)sim.objects[i].type) {
      case sim_perception::ObstacleType::CAR:obj.type = caic_perception::ObstacleType::VEHICLE;
        obj.veh_sub_type = caic_perception::VehicleSubType::CAR;
        break;
      case sim_perception::ObstacleType::TRUCK:obj.type = caic_perception::ObstacleType::VEHICLE;
        obj.veh_sub_type = caic_perception::VehicleSubType::TRUCK;
        break;
      case sim_perception::ObstacleType::VAN:obj.type = caic_perception::ObstacleType::VEHICLE;
        obj.veh_sub_type = caic_perception::VehicleSubType::VAN;
        break;
      case sim_perception::ObstacleType::BUS:obj.type = caic_perception::ObstacleType::VEHICLE;
        obj.veh_sub_type = caic_perception::VehicleSubType::BUS;
        break;
      case sim_perception::ObstacleType::BIKE:obj.type = caic_perception::ObstacleType::CYCLIST;
        obj.cyclist_sub_type = caic_perception::CyclistSubType::BICYCLE;
        break;
      case sim_perception::ObstacleType::MOTORBIKE:obj.type = caic_perception::ObstacleType::CYCLIST;
        obj.cyclist_sub_type = caic_perception::CyclistSubType::MOTORCYCLE;
        break;
      case sim_perception::ObstacleType::PEDESTRIAN:obj.type = caic_perception::ObstacleType::PEDESTRIAN;
        break;
      case sim_perception::ObstacleType::TRAFFIC_SIGN:obj.type = caic_perception::ObstacleType::TRAFFIC_SIGN;
        break;
      case sim_perception::ObstacleType::BARRIER:obj.type = caic_perception::ObstacleType::TRAFFIC_BARRIER;
        break;
      default:obj.type = caic_perception::ObstacleType::UNKNOWN;
        break;
    }
    obj.valid = 1;
    obj.pose_in_world.position.x = sim.objects[i].pose_in_world.position.x;
    obj.pose_in_world.position.y = sim.objects[i].pose_in_world.position.y;
    obj.pose_in_world.position.z = sim.objects[i].pose_in_world.position.z;
    if (std::hypot(sim.objects[i].pose_in_world.velocity.x, sim.objects[i].pose_in_world.velocity.y) < 1.0) {
      obj.pose_in_world.velocity.x = 0.0;
      obj.pose_in_world.velocity.y = 0.0;
    } else {
      obj.pose_in_world.velocity.x = sim.objects[i].pose_in_world.velocity.x;
      obj.pose_in_world.velocity.y = sim.objects[i].pose_in_world.velocity.y;
    }
    obj.pose_in_world.heading = sim.objects[i].pose_in_world.heading;
    obj.pose_in_world.position_reliable = 1;
    obj.pose_in_world.velocity_reliable = 1;
    obj.pose_in_world.acceleration_reliable = 0;
    obj.pose_in_vcs.position.x = sim.objects[i].pose_in_vcs.position.x;
    obj.pose_in_vcs.position.y = sim.objects[i].pose_in_vcs.position.y;
    obj.pose_in_vcs.position.z = sim.objects[i].pose_in_vcs.position.z;
    if (std::hypot(sim.objects[i].pose_in_vcs.velocity.x, sim.objects[i].pose_in_vcs.velocity.y) < 1.0) {
      obj.pose_in_vcs.velocity.x = 0.0;
      obj.pose_in_vcs.velocity.y = 0.0;
    } else {
      obj.pose_in_vcs.velocity.x = sim.objects[i].pose_in_vcs.velocity.x;
      obj.pose_in_vcs.velocity.y = sim.objects[i].pose_in_vcs.velocity.y;
    }
    obj.pose_in_vcs.heading = sim.objects[i].pose_in_vcs.heading;
    obj.pose_in_vcs.position_reliable = 1;
    obj.pose_in_vcs.velocity_reliable = 1;
    obj.pose_in_vcs.acceleration_reliable = 0;
    obj.length = sim.objects[i].length;
    obj.width = sim.objects[i].width;
    obj.height = sim.objects[i].height;
    // set avaliable
    obj.avaliable = 0;
    obj.avaliable |= obj.PERCEPTION_OBJECT_TYPE;
    obj.avaliable |= obj.PERCEPTION_OBJECT_VALID;
    obj.avaliable |= obj.PERCEPTION_OBJECT_POSE_IN_WORLD;
    obj.avaliable |= obj.PERCEPTION_OBJECT_POSE_IN_VCS;
    obj.avaliable |= obj.PERCEPTION_OBJECT_LENGTH;
    obj.avaliable |= obj.PERCEPTION_OBJECT_WIDTH;
    obj.avaliable |= obj.PERCEPTION_OBJECT_HEIGHT;
    obj.avaliable |= obj.PERCEPTION_OBJECT_COVARIANCE;
    output.objects[objects_size] = obj;
    objects_size++;
    if(objects_size > OBJ_SIZE) {
      LOG_ERROR("front vision veh objects size > OBJ_SIZE");
      exit(-1);
    }
    output.objects_size = objects_size;
  }
}
} // namespace
