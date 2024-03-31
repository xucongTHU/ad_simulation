// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#pragma once

// #include "app/common.h"
#include "ad_interface.h"
#include "cm/cm.h"
// #include "common/magic.h"
// #include "msgs/msgs.h"
#include "pattern/observer.hpp"
#include "pattern/task.hpp"

namespace stoic::app::codec {
class CameraLoader : public stoic::cm::pattern::Task<CameraLoader>,
                     public stoic::cm::pattern::Subject<caic_sensor::Image>,
                     public stoic::cm::pattern::Observer<caic_sensor::Image> {
 public:
  template <typename... OptionArgs>
  CameraLoader(cm::NodeHandle& nh, OptionArgs&&... option_args)
      : Task(nh, std::forward<OptionArgs>(option_args)...) {}

  CameraLoader(cm::NodeHandle& nh, const stoic::cm::pattern::TaskOptions& task_options)
      : Task(nh, task_options) {}

  void callback(const caic_sensor::Image&, const std::string&);
};

}  // namespace stoic::app::codec

STOIC_WORKFLOW_ADD_TASK(::stoic::app::codec::CameraLoader, caic_sensor::Image, caic_sensor::Image)
