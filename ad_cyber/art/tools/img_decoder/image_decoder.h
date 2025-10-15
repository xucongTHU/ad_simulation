// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#pragma once

#include <pthread.h>
#include <sys/mman.h>
#include "../gpu_jpeg/gpu_codec.h"
#include "ad_interface.h"
#include "cm/cm.h"
#include "pattern/observer.hpp"
#include "pattern/task.hpp"
#include "proto/common/string.pb.h"
#include "proto/sensor/image.pb.h"

using ImageH264 = stoic::cm::proto::sensor::ImageCodec;

namespace stoic::app::codec {

class ImageDecoder : public stoic::cm::pattern::Task<stoic::app::codec::ImageDecoder>,
                     public stoic::cm::pattern::Subject<caic_sensor::Image>,
                     public stoic::cm::pattern::Observer<stoic::cm::proto::sensor::ImageCodec> {
 public:
  template <typename... OptionArgs>
  ImageDecoder(::stoic::cm::NodeHandle& nh, OptionArgs&&... option_args)
      : Task(nh, std::forward<OptionArgs>(option_args)...) {
    nh_ptr_ = &nh;
  }

  ImageDecoder(::stoic::cm::NodeHandle& nh, const stoic::cm::pattern::TaskOptions& task_options)
      : Task(nh, task_options) {
    nh_ptr_ = &nh;
  }

  ~ImageDecoder() {}
  void callback(const stoic::cm::proto::sensor::ImageCodec&, const std::string&);

 private:
  ::stoic::cm::NodeHandle* nh_ptr_;
  bool is_init_ = 0;
  std::shared_ptr<GPUDecoder> gpu_decoder_;
  caic_sensor::Image im_out;

  bool convert_h264msg(std::shared_ptr<stoic::cm::proto::sensor::ImageCodec> msg_imgh264,
                       caic_sensor::Image& msg);
};
}  // namespace stoic::app::codec

STOIC_WORKFLOW_ADD_TASK(::stoic::app::codec::ImageDecoder, caic_sensor::Image,
                        stoic::cm::proto::sensor::ImageCodec)
