#pragma once
#include <memory>

#include "Eigen/Dense"
#include "Eigen/Geometry"


#include "caic_interface/caic_std.h"


namespace simulator {
namespace vtd_integration {

struct Point{
  Point(float x_in, float y_in):x(x_in),y(y_in){}
  float x;
  float y;
};

struct PoseStamped {
  double t;  // s
//  manif::SE3d precise_pose;
//  manif::SE3d esitmated_pose;
  PoseStamped() : t(0.0) {}
};

struct PlanningPose {
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
  PlanningPose() {
    rotation.setIdentity();
    translation.setZero();
  }

  PlanningPose Inverse() {
    PlanningPose pose;
    pose.rotation = rotation.inverse();
    pose.translation = -translation;
    return pose;
  }
};

using PoseStampedPtr = std::shared_ptr<PoseStamped>;
}
}
