// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#pragma once

// #include "app/common.h"
#include "ad_interface.h"
#include "cm/cm.h"
// #include "msgs/msgs.h"
#include "pattern/observer.hpp"
#include "pattern/task.hpp"

// #include "proto/sensor/image.pb.h"

namespace stoic::app::codec {
class ImageEncoderDriver : public stoic::cm::pattern::Task<ImageEncoderDriver>,
                           public stoic::cm::pattern::Subject<caic_sensor::Image> {
 public:
  template <typename... OptionArgs>
  ImageEncoderDriver(cm::NodeHandle& nh, OptionArgs&&... option_args)
      : Task(nh, std::forward<OptionArgs>(option_args)...) {}

  ImageEncoderDriver(cm::NodeHandle& nh, const stoic::cm::pattern::TaskOptions& task_options)
      : Task(nh, task_options) {}
};

}  // namespace stoic::app::codec

STOIC_WORKFLOW_ADD_TASK(::stoic::app::codec::ImageEncoderDriver, caic_sensor::Image)
