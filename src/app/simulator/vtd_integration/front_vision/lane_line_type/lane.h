#pragma once

#include <math.h>
#include <memory>
#include "simulator/vtd_integration/front_vision/lane_line_type/lane_boundary.h"
#include "simulator/vtd_integration/front_vision/lane_line_type/lane_fuision_data_type.h"
#include "common/log/Logger.h"

// #include <cassert>

namespace simulator {
namespace vtd_integration {

using caic_perception::PerceptionLane;

constexpr double kValidScore = 0.7;



class Lane {
 public:
  void push(const LaneIndex& index, LaneBoundaryPtr left, LaneBoundaryPtr right){
    SetIndex(index);
    if (left->Valid()){
      left_.emplace_back(left);
    }
    if (right->Valid()){
      right_.emplace_back(right);
    }
    if (!left_.empty() || !right_.empty()){
      SetValid(true);
    }
    else{
      SetValid(false);
    }
  }
  Lane(const LaneIndex& index, LaneBoundaryPtr left, LaneBoundaryPtr right);
  Lane(){}
  ~ Lane();
 public:
  // std::string Id() const;

  void RectifierLeftRightBoundary();  // 整流左右边界，保持两边界线段组size一致

  bool Valid(){
    // if (LeftBoundaryValid() && RightBoundaryValid()){  // 保证两边车道均有效
    //   valid_ = true;
    //   return valid_;
    // }
    // else{
    //   valid_ = false;
      return valid_;
    // }
  }

  std::shared_ptr<Lane> LeftLane() { return left_lane_.lock(); }
  std::shared_ptr<Lane> RightLane() { return right_lane_.lock(); }
  const std::shared_ptr<Lane> LeftLane() const { return left_lane_.lock(); }
  const std::shared_ptr<Lane> RightLane() const { return right_lane_.lock(); }
  void SetLeftLane(const std::shared_ptr<Lane>& lane) {
    left_lane_ = lane;
    left_left_ = lane->LeftBoundary();
  }

  void SetRightLane(const std::shared_ptr<Lane>& lane) {
    right_lane_ = lane;
    right_right_ = lane->RightBoundary();
  }

  std::vector<LaneBoundaryPtr> LeftBoundary() { return left_; }
  bool LeftBoundaryValid() {
    if (left_.empty()){
      return false;
    }
    else{
      int size = left_.size();
      int valid_count = 0;
      for (int i = 0; i < size; i ++){
        if(left_[i]->Valid()){
          valid_count ++;
        }
      }
      if (valid_count > 0){
        return true;
      }
      else{
        return false;
      }
    }
  }

  bool LeftLeftBoundaryValid() {
    if (left_left_.empty()){
      return false;
    }
    else{
      int size = left_left_.size();
      int valid_count = 0;
      for (int i = 0; i < size; i ++){
        if(left_left_[i]->Valid()){
          valid_count ++;
        }
      }
      if (valid_count > 0){
        return true;
      }
      else{
        return false;
      }
    }
  }

  bool RightBoundaryValid() {
    if (right_.empty()){
      return false;
    }
    else{
      int size = right_.size();
      int valid_count = 0;
      for (int i = 0; i < size; i ++){
        if(right_[i]->Valid()){
          valid_count ++;
        }
      }
      if (valid_count > 0){
        return true;
      }
      else{
        return false;
      }
    }
  }

  bool RightRightBoundaryValid() {
    if (right_right_.empty()){
      return false;
    }
    else{
      int size = right_right_.size();
      int valid_count = 0;
      for (int i = 0; i < size; i ++){
        if(right_right_[i]->Valid()){
          valid_count ++;
        }
      }
      if (valid_count > 0){
        return true;
      }
      else{
        return false;
      }
    }
  }

  const std::vector<LaneBoundaryPtr> LeftBoundary() const { return left_; }
  std::vector<LaneBoundaryPtr> RightBoundary() { return right_; }
  const std::vector<LaneBoundaryPtr> RightBoundary() const { return right_; }

  std::vector<LaneBoundaryPtr> LeftLeftBoundary() { return left_left_; }
  const std::vector<LaneBoundaryPtr> LeftLeftBoundary() const { return left_left_; }
  std::vector<LaneBoundaryPtr> RightRightBoundary() { return right_right_; }
  const std::vector<LaneBoundaryPtr> RightRightBoundary() const { return right_right_; }

  void ResampleLaneSegments();

  void FitBoundarySegments(std::vector<simulator::vtd_integration::LaneBoundaryPtr> &);

  void FitLaneSegments();

  // cubic_polynomial Centerline(const bool& use_filter);
  void SetValid(const bool& valid) { valid_ = valid;}
 private:
  void SetIndex(const LaneIndex index) { index_ = index;}


 private:
  LaneIndex index_;
  std::vector<LaneBoundaryPtr> left_;
  std::vector<LaneBoundaryPtr> right_;
  std::vector<LaneBoundaryPtr> left_left_;
  std::vector<LaneBoundaryPtr> right_right_;

  std::weak_ptr<Lane> left_lane_;
  std::weak_ptr<Lane> right_lane_;

  bool valid_ = false;
};

}
}


