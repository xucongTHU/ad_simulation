
//
// Created by xucong on 24-1-3.
//
#ifndef SIM_PERCEPTION_FRONTVISION_COMMON_H_
#define SIM_PERCEPTION_FRONTVISION_COMMON_H_
#include <iostream>
#include <vector>
#include <manif/manif.h>

const manif::SE3d TX_I_V(0,0,0,0,0,-M_PI_2);
const manif::SE3d TX_V_I(0,0,0,0,0,M_PI_2);
double getEgoYaw(const manif::SE3d& egoPos)
{
  Eigen::Vector3d p1(1, 0, 0);
  manif::SE3d tempTf = egoPos.compose(TX_V_I);
  Eigen::Vector3d tf_point = tempTf.act(p1);
  tf_point=tf_point-tempTf.translation();
  return std::atan2(tf_point.y(),tf_point.x());
}
double getEgoPitch(const manif::SE3d& egoPos)
{
  Eigen::Vector3d p1(1, 0, 0);
  manif::SE3d tempTf = egoPos.compose(TX_V_I);
  Eigen::Vector3d tf_point = tempTf.act(p1);
  tf_point=tf_point-tempTf.translation();
  return std::atan2(tf_point.z(),std::hypot(tf_point.x(),tf_point.y()));
}
double getEgoRoll(const manif::SE3d& egoPos)
{
  Eigen::Vector3d p1(0, -1, 0);
  manif::SE3d tempTf = egoPos.compose(TX_V_I);
  Eigen::Vector3d tf_point = tempTf.act(p1);
  tf_point=tf_point-tempTf.translation();
  return std::atan2(tf_point.z(),std::hypot(tf_point.x(),tf_point.y()));
}
// vcs to map
void getV2MTransform(const manif::SE3d& egoPos,manif::SE3d& TX_V_M)
{
  TX_V_M = egoPos*TX_V_I;
}
// map to vcs
void getM2VTransform(const manif::SE3d& egoPos,manif::SE3d& TX_M_V)
{
  manif::SE3d TX_M_I = egoPos.inverse();
  TX_M_V = TX_I_V*TX_M_I;
}

struct BoundingBox {
  float x;
  float y;
  float w;
  float h;
};
struct Detection {
  float score;       // confidence
  BoundingBox bbox;  // bbox
  int32_t id;        // classes index
};
struct DetectResult {
  float score;  // confidence
  int32_t x1;   // top
  int32_t y1;   // left
  int32_t x2;   // bottom
  int32_t y2;   // right
  int32_t id;   // classes index
  float area;   // bbox area
  DetectResult() : score{0.f}, x1(0), y1(0), x2(0), y2(0), id(-1), area(0.f) {}
};
struct DetectResultTl {
  float score;  // detect confidence
  int32_t x1;   // top
  int32_t y1;   // left
  int32_t x2;   // bottom
  int32_t y2;   // right
  int32_t id;   // classes index
  float time;   // time
  int32_t color;
  DetectResultTl() : score{0.f}, x1(0), y1(0), x2(0), y2(0), id(-1), time(0), color(-1){}
};
struct DetectResultTls {
  std::vector<DetectResultTl> tl_results_one_frame;
  int64_t stamp;
};
typedef struct KPResult {
  float fscore;    // front keypoint confidence
  float bscore;    // back keypoint confidence
  int32_t frontx;  // front keypoint x
  int32_t fronty;  // front keypoint y
  int32_t backx;   // back keypoint x
  int32_t backy;   // back keypoint y
  KPResult() : fscore{0.f}, bscore{0.f}, frontx(0), fronty(0), backx(0), backy(0) {}
}KPResult_t;
// model cfgs
struct ModelCfg {
  std::string cfg_4pe_detect;
  std::string cfg_4pe_seg;
  std::string cfg_2pe_veh;
  std::string cfg_2pe_rear;
  std::string cfg_2pe_ped;
  std::string cfg_2pe_cyc;
  std::string cfg_2pe_sign;
};
#endif //SIM_PERCEPTION_FRONTVISION_COMMON_H_
