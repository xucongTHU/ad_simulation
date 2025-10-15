// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#include "camera_loader.h"
namespace stoic::app::codec {
void CameraLoader::callback(const stoic::cm::proto::sensor::ImageCodec& msg,
                            const std::string& topic) {
  if (0) {
    std::cout << "CameraLoader, topic is:" << topic << ", topicName() is: " << topicName()
              << std::endl;
  }
  notifyAll(msg, topicName());
}
}  // namespace stoic::app::codec
