// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.
#pragma once

// #include "ThreadPool/ThreadPool.h"
#include "camera_loader.h"
#include "cm/cm.h"
#include "image_encoder.h"
#include "image_encoder_driver.h"
#include "pattern/workflow.hpp"

using namespace stoic;

class WorkflowLoader {
 public:
  WorkflowLoader(stoic::cm::NodeHandle& nh, const std::string& deployment,
                 std::string& channel_in_name)
      : nh_(nh), deployment_(deployment), channel_in_name_(channel_in_name) {}

  void init(stoic::cm::pattern::Workflow* wf) {
    // thread_pool_.reset(new ThreadPool(1));

    front_wide_driver = new ::stoic::app::codec::ImageEncoderDriver(
        nh_, "Image Encoder Driver", "DRI", "driver", "camera", channel_in_name_);

    front_wide_loader = new ::stoic::app::codec::CameraLoader(
        nh_, "Image Encoder Driver", "PERC:ALL", "loader", "camera", channel_in_name_);

    encoder =
        new ::stoic::app::codec::ImageEncoder(nh_, "Image Encoder", "PERC:ALL", "loader", "camera",
                                              "/front_encoder", "lightgray", deployment_);

    wf->addLink(front_wide_driver, front_wide_loader, deployment_);

    wf->addLink(front_wide_loader, encoder, deployment_);

    // thread_pool_->enqueue(boost::bind(&stoic::app::codec::ImageEncoder::run, this->encoder));
  }

  ~WorkflowLoader() {
    delete front_wide_driver;
    delete front_wide_loader;
    delete encoder;
  }

 private:
  stoic::cm::NodeHandle& nh_;
  std::string deployment_;
  // std::unique_ptr<ThreadPool> thread_pool_;

  std::string channel_in_name_;
  ::stoic::app::codec::ImageEncoderDriver* front_wide_driver;
  ::stoic::app::codec::CameraLoader* front_wide_loader;
  ::stoic::app::codec::ImageEncoder* encoder;
};
