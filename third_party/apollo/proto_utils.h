// Copyright 2021 The  .COM Authors. All Rights Reserved.

#pragma once

#include <google/protobuf/message.h>
#include <string>

namespace smartsim {

bool loadFromFile(const std::string& filename, ::google::protobuf::Message* message);

bool saveToFile(const std::string& filename, const ::google::protobuf::Message& message);

}  // namespace smartcar::common::proto
