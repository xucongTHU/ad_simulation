// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#pragma once

#include "ad_interface.h"
#include "cm/cm.h"
#include "pattern/observer.hpp"
#include "pattern/task.hpp"

#include "proto/sensor/image.pb.h"

namespace stoic::app::codec {
class CameraDriver : public stoic::cm::pattern::Task<CameraDriver>,
                     public stoic::cm::pattern::Subject<stoic::cm::proto::sensor::ImageCodec> {
 public:
  template <typename... OptionArgs>
  CameraDriver(cm::NodeHandle& nh, OptionArgs&&... option_args)
      : Task(nh, std::forward<OptionArgs>(option_args)...) {}

  CameraDriver(cm::NodeHandle& nh, const stoic::cm::pattern::TaskOptions& task_options)
      : Task(nh, task_options) {}
};

}  // namespace stoic::app::codec

STOIC_WORKFLOW_ADD_TASK(::stoic::app::codec::CameraDriver, stoic::cm::proto::sensor::ImageCodec)
