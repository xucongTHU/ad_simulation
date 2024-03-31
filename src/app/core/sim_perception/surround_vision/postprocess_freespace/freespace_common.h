
//
// Copyright (c) 2024 xucong Authors. All rights reserved.
// Created by xucong on 24-3-15.
//
#pragma once
#include <iostream>
#include <memory>
#include <vector>

struct FreespacePoint {
  int type;
  // cv::Point point;
  float ipm_x;
  float ipm_y;
  float vcs_x;
  float vcs_y;
};
// freespace
struct FreespaceResult {
  using Ptr = std::shared_ptr<FreespaceResult>;
  using ConstPtr = std::shared_ptr<const FreespaceResult>;
  std::vector<FreespacePoint> fs_points_;
};
struct Freespace_TENSOR_OUT{
  float* surround;
  int* type_surround;
};
struct FreespaceObstacle {
  float vcs_x;
  float vcs_y;
  FreespaceObstacle(float x, float y) : vcs_x(0), vcs_y(0) {}
};
