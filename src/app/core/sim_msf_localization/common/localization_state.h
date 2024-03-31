//
// Created by xucong on 23-12-26.
//

#ifndef LOCALIZATION_STATE_H_
#define LOCALIZATION_STATE_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace stoic::app::core {
class LocalizationState {
 public:
  LocalizationState() {
    pos_wi_.setZero();
    vel_wi_.setZero();
    acc_wi_.setZero();
    rpy_wi_.setZero();
    avel_wi_.setZero();
    Rwi_.setIdentity();
  };

 public:
  Eigen::Vector3d pos_wi_;
  Eigen::Vector3d vel_wi_;
  Eigen::Vector3d acc_wi_;
  Eigen::Vector3d rpy_wi_;
  Eigen::Vector3d avel_wi_;
  Eigen::Matrix3d Rwi_;

  double timestamp;
  double heading;
  Eigen::Quaterniond qt;


};
}

#endif //LOCALIZATION_STATE_H_
