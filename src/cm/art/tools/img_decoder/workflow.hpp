// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.
#pragma once

#include "camera_driver.h"
#include "camera_loader.h"
#include "cm/cm.h"
#include "image_decoder.h"
#include "pattern/workflow.hpp"

using namespace stoic;

class WorkflowLoader {
 public:
  WorkflowLoader(stoic::cm::NodeHandle& nh, const std::string& deployment,
                 std::string& channel_in_name)
      : nh_(nh), deployment_(deployment), channel_in_name_(channel_in_name) {}

  void init(stoic::cm::pattern::Workflow* wf) {
    camera_driver = new ::stoic::app::codec::CameraDriver(nh_, "Image Encoder Driver", "DRI",
                                                          "driver", "camera", channel_in_name_);
    camera_loader = new ::stoic::app::codec::CameraLoader(nh_, "Image Encoder Driver", "PERC:ALL",
                                                          "loader", "camera", channel_in_name_);
    decoder = new ::stoic::app::codec::ImageDecoder(nh_, "Image Encoder", "PERC:ALL", "loader",
                                                    "camera", "/decoder", "lightgray", deployment_);

    wf->addLink(camera_driver, camera_loader, deployment_);

    wf->addLink(camera_loader, decoder, deployment_);
  }

  ~WorkflowLoader() {
    delete camera_driver;
    delete camera_loader;
    delete decoder;
  }

 private:
  stoic::cm::NodeHandle& nh_;
  std::string deployment_;
  std::string channel_in_name_;
  ::stoic::app::codec::CameraDriver* camera_driver;
  ::stoic::app::codec::CameraLoader* camera_loader;
  ::stoic::app::codec::ImageDecoder* decoder;
};
