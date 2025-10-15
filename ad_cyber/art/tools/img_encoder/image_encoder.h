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

struct CompressedImg {
  caic_std::Header header;
  uint32_t width;
  uint32_t height;
  uint32_t size;
  std::vector<uint8_t> data;
};
using CompressedImgPtr = std::shared_ptr<CompressedImg>;

class ImageEncoder : public stoic::cm::pattern::Task<stoic::app::codec::ImageEncoder>,
                     public stoic::cm::pattern::Subject<caic_sensor::Image>,
                     public stoic::cm::pattern::Observer<caic_sensor::Image> {
 public:
  template <typename... OptionArgs>
  ImageEncoder(::stoic::cm::NodeHandle& nh, OptionArgs&&... option_args)
      : Task(nh, std::forward<OptionArgs>(option_args)...) {
    nh_ptr_ = &nh;
  }

  ImageEncoder(::stoic::cm::NodeHandle& nh, const stoic::cm::pattern::TaskOptions& task_options)
      : Task(nh, task_options) {
    nh_ptr_ = &nh;
  }

  ~ImageEncoder() {}
  void callback(const caic_sensor::Image&, const std::string&);

  void run();
  // CompressedImgPtr pop();

 private:
  ::stoic::cm::NodeHandle* nh_ptr_;
  bool is_init_ = 0;

  // stoic::TimeSeriesBuffer<CompressedImgPtr> frame_buffer_;
  // std::mutex frame_buffer_mutex_;

  std::shared_ptr<GPUEncoder> gpu_encoder_;
  void convert_msg_h264(const caic_sensor::Image& msg, std::shared_ptr<ImageH264>& msg_imgh264);
  void convert_msg_h264(CompressedImg& msg, std::shared_ptr<ImageH264>& msg_imgh264);
};
}  // namespace stoic::app::codec

STOIC_WORKFLOW_ADD_TASK(::stoic::app::codec::ImageEncoder, caic_sensor::Image, caic_sensor::Image)
