// Copyright 2021 The  .COM Authors. All Rights Reserved.

#include "third_party/smartsim/proto_utils.h"
#include <fcntl.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <fstream>
#include <iostream>

namespace smartsim {

bool loadFromTextFile(const std::string& filename, ::google::protobuf::Message* message) {
  int32_t fd = open(filename.c_str(), O_RDONLY);
  if (fd < 0) {
    std::cerr << "Load protobuf file failed: " << filename << std::endl;
    return false;
  }
  ::google::protobuf::io::FileInputStream fis(fd);
  fis.SetCloseOnDelete(true);
  if (!::google::protobuf::TextFormat::Parse(&fis, message)) {
    std::cerr << "Load protobuf file failed: " << filename << std::endl;
    return false;
  }
  std::cout << "Load protobuf file succeeded: " << filename << std::endl;
  return true;
}

bool loadFromBinaryFile(const std::string& filename, ::google::protobuf::Message* message) {
  std::fstream fs(filename.c_str(), std::ios::in | std::ios::binary);
  if (!message->ParseFromIstream(&fs)) {
    std::cerr << "Load protobuf file failed: " << filename << std::endl;
    return false;
  }
  std::cout << "Load protobuf file succeeded: " << filename << std::endl;
  return true;
}

bool saveToTextFile(const std::string& filename, const ::google::protobuf::Message& message) {
  int32_t fd = open(filename.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) {
    std::cerr << "Save protobuf file failed: " << filename << std::endl;
    return false;
  }
  ::google::protobuf::io::FileOutputStream fos(fd);
  fos.SetCloseOnDelete(true);
  if (!::google::protobuf::TextFormat::Print(message, &fos)) {
    std::cerr << "Save protobuf file failed: " << filename << std::endl;
    return false;
  }
  std::cout << "Save protobuf file succeeded: " << filename << std::endl;
  return true;
}

bool saveToBinaryFile(const std::string& filename, const ::google::protobuf::Message& message) {
  std::fstream fs(filename.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
  if (!message.SerializeToOstream(&fs)) {
    std::cerr << "Save protobuf file failed: " << filename << std::endl;
    return false;
  }
  std::cout << "Save protobuf file succeeded: " << filename << std::endl;
  return true;
}

bool loadFromFile(const std::string& filename, ::google::protobuf::Message* message) {
  if (filename.empty()) {
    std::cerr << "Load protobuf file failed, filename empty." << std::endl;
    return false;
  }
  char type = filename.back();
  switch (type) {
    case 't':  // "txt"
    case 'g':  // "cfg"
      return loadFromTextFile(filename, message);
    case 'n':  // "bin"
      return loadFromBinaryFile(filename, message);
  }
  return false;
}

bool saveToFile(const std::string& filename, const ::google::protobuf::Message& message) {
  if (filename.empty()) {
    std::cerr << "Save protobuf file failed, filename empty." << std::endl;
    return false;
  }
  char type = filename.back();
  switch (type) {
    case 't':  // "txt"
    case 'g':  // "cfg"
      return saveToTextFile(filename, message);
    case 'n':  // "bin"
      return saveToBinaryFile(filename, message);
  }
  return false;
}

} // namespace
