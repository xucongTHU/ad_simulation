#include "app/simulator/vtd_integration/front_vision/lane_line_type/lane.h"

namespace simulator {
namespace vtd_integration {

Lane::Lane(const LaneIndex& index, LaneBoundaryPtr left, LaneBoundaryPtr right)
      : index_(index){
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
Lane::~Lane(){}

void Lane::RectifierLeftRightBoundary(){
  /*根据左右车道线的最小线段组数，对车道线进行整流，并设置标志位Valid*/
  /*函数应放置在拟合车道线之后*/
  int left_size = left_.size();
  int right_size = right_.size();
  int valid_count = 0;

  // 滤除边线size为0的线段
  if (left_size < 1 || right_size < 1){
    left_.clear();
    right_.clear();
    SetValid(false);  // lane::valid标志位设置为false
    return;
  }
  else{
    if (left_size == right_size){
      for (int i = 0; i < left_size; i++){
        if (left_[i]->Valid() && right_[i]->Valid()){
          valid_count++;
          continue;   // 有效性相同时，保持不变
        }
        else{  // 有效性不同时，全部设为无效，保证左右两边线段成对输出
          left_[i]->SetValid(false);
          right_[i]->SetValid(false);
        }
       }
    }
    else{
    SetValid(false);  // 左右线段个数不一致，lane::valid标志位设置为false
    return;
    }
  }
  if(valid_count<1){
    SetValid(false);  // 有效线对个数小于1，lane::valid设置为false
  }
  else{
    SetValid(true);  // 有效线对个数不小于1，lane::valid设置为true
  }
}

void Lane::ResampleLaneSegments(){
  // lane lines resample use raw line parameters
  // auto left_boundary = LeftBoundary();
  // auto right_boundary = RightBoundary();
  // auto left_left_boundary = LeftLane()->LeftBoundary();
  // auto right_right_boundary = RightLane()->RightBoundary();
  // // if(left_boundary->GetRawLine() == )
  if (!left_.empty()){
    for (LaneBoundaryPtr lft_bd: LeftBoundary()){
      lft_bd->Resample();
    }
  }
  if (!right_.empty()){
    for (LaneBoundaryPtr rt_bd: RightBoundary()){
      rt_bd->Resample();
    }
  }
  if (!left_left_.empty()){
    for (LaneBoundaryPtr lft_lft_bd: LeftLeftBoundary()){
      lft_lft_bd->Resample();
    }
  }
  if (!right_right_.empty()){
    for (LaneBoundaryPtr rt_rt_bd: RightRightBoundary()){
      rt_rt_bd->Resample();
    }
  }
}

// void Lane::FitLaneSegments(){
//   LeftBoundary()->FitPoly();
//   RightBoundary()->FitPoly();
//   LeftLeftBoundary()->FitPoly();
//   RightRightBoundary()->FitPoly();
// }

void Lane::FitBoundarySegments(std::vector<simulator::vtd_integration::LaneBoundaryPtr> &boundaries){
  /*对多段线进行策略拟合*/
  /*将后一线段中的起始点添加到前一线段中，参与拟合*/
  if (boundaries.size() < 2){
    boundaries[0]->FitPoly(); // 单线段直接拟合
    return;
  }
  else{
    int size_boundaries = boundaries.size();
    for (int i =0; i < size_boundaries; i++){
      if (i == size_boundaries - 1){
        boundaries[i]->FitPoly(); // 末端线段直接拟合
        return;
      }

      // 更新下一段线的起点到此段线的终点，更新end_x的值，并进行拟合
      caic_std::Point2d point_1 = boundaries[i+1]->GetPoint(0);
      caic_std::Point2d point_2 = boundaries[i+1]->GetPoint(1);
      caic_std::Point2d point_3 = boundaries[i+1]->GetPoint(2);
      boundaries[i]->PushAdditionPoint(point_1);
      boundaries[i]->PushAdditionPoint(point_2);
      boundaries[i]->PushAdditionPoint(point_3);

      double x = boundaries[i+1]->GetStartX();
      boundaries[i]->SetEndX(x);

      // 更新上一段线的终点到此段线的起点
      if (i > 0){
        caic_std::Point2d point_1 = boundaries[i-1]->GetEndPoint(0);
        caic_std::Point2d point_2 = boundaries[i-1]->GetEndPoint(1);
        caic_std::Point2d point_3 = boundaries[i-1]->GetEndPoint(2);
        boundaries[i]->PushAdditionPoint(point_1);
        boundaries[i]->PushAdditionPoint(point_2);
        boundaries[i]->PushAdditionPoint(point_3);
      }

      boundaries[i]->FitPoly();
    }
  }
}

void Lane::FitLaneSegments(){
  if (!left_.empty()){
    FitBoundarySegments(left_);
  }
  if (!right_.empty()){
    FitBoundarySegments(right_);
  }
  if (!left_left_.empty()){
    FitBoundarySegments(left_left_);
  }
  if (!right_right_.empty()){
    FitBoundarySegments(right_right_);
  }
  RectifierLeftRightBoundary();
}

}
}
