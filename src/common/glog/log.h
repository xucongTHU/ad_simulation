// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#pragma once

#include <boost/filesystem.hpp>
#include <cstdarg>
#include <iostream>
#include <string>

#include "glog/logging.h"
#include "glog/raw_logging.h"

namespace bf = ::boost::filesystem;

#define LOG_INFO LOG_MODULE(INFO)
#define LOG_WARN LOG_MODULE(WARN)
#define LOG_ERROR LOG_MODULE(ERROR)
#define LOG_FATAL LOG_MODULE(FATAL)

#define DLOG_INFO LOG_MODULE(DINFO)
#define DLOG_WARN LOG_MODULE(DWARN)
#define DLOG_ERROR LOG_MODULE(DERROR)
#define DLOG_FATAL LOG_MODULE(DFATAL)

#ifndef LOG_MODULE
#define LOG_MODULE(log_severity) LOG_MODULE_STREAM_##log_severity
#endif

#define LOG_MODULE_STREAM_INFO google::LogMessage(__FILE__, __LINE__, google::INFO).stream()

#define LOG_MODULE_STREAM_WARN google::LogMessage(__FILE__, __LINE__, google::WARNING).stream()

#define LOG_MODULE_STREAM_ERROR google::LogMessage(__FILE__, __LINE__, google::ERROR).stream()

#define LOG_MODULE_STREAM_FATAL google::LogMessage(__FILE__, __LINE__, google::FATAL).stream()

#ifdef DEBUG
#define LOG_MODULE_STREAM_DINFO LOG_MODULE_STREAM_INFO

#define LOG_MODULE_STREAM_DWARN LOG_MODULE_STREAM_WARN

#define LOG_MODULE_STREAM_DERROR LOG_MODULE_STREAM_ERROR

#define LOG_MODULE_STREAM_DFATAL LOG_MODULE_STREAM_FATAL
#else
#define LOG_MODULE_STREAM_DINFO \
  static_cast<void>(0), true ? (void)0 : google::LogMessageVoidify() & LOG_MODULE_STREAM_INFO

#define LOG_MODULE_STREAM_DWARN \
  static_cast<void>(0), true ? (void)0 : google::LogMessageVoidify() & LOG_MODULE_STREAM_WARN

#define LOG_MODULE_STREAM_DERROR \
  static_cast<void>(0), true ? (void)0 : google::LogMessageVoidify() & LOG_MODULE_STREAM_ERROR

#define LOG_MODULE_STREAM_DFATAL \
  static_cast<void>(0), true ? (void)0 : google::LogMessageVoidify() & LOG_MODULE_STREAM_FATAL
#endif

inline std::vector<std::string> split(const std::string& str, const std::string& symbol) {
  std::vector<std::string> res;
  std::string temp = str;
  size_t pos;
  while ((pos = temp.find_first_of(symbol)) != std::string::npos) {
    std::string t = temp.substr(0, pos);
    if (!t.empty() && t != "") res.push_back(t);
    temp = temp.substr(pos + 1);
  }
  if (!temp.empty() && temp != "") res.push_back(temp);
  return res;
}

inline void StartLogging(const std::string& module_dir, const std::string& log_dir_prefix) {
  std::vector<std::string> date_vec = split(module_dir, "/");
  if (date_vec.empty()) {
    std::cout << "module dir does not correct: " << module_dir << std::endl;
    return;
  }
  std::string log_dir = log_dir_prefix + "/" + date_vec.back();
  if (!bf::exists(log_dir)) {
    bf::create_directories(log_dir);
  }
  FLAGS_log_dir = log_dir.c_str();

  FLAGS_minloglevel = google::INFO; /**< Log level[INFO(0) || WARNING(1) || ERROR(2) || FATAL(3)]*/
  FLAGS_alsologtostderr = true;     /**< Set whether log messages go to stderr and logfiles*/
  FLAGS_colorlogtostderr = true;    /**< Set color messages logged to stderr */
  FLAGS_max_log_size = 15;          /**<  Sets the maximum log file size (in MB). */

  // google::SetLogDestination(google::WARNING, "");
  // google::SetLogDestination(google::ERROR, "");
  // google::SetLogDestination(google::FATAL, "");
  google::InitGoogleLogging(module_dir.c_str());
  // google::SetStderrLogging(google::GLOG_INFO);

  LOG_INFO << "GLAGS_log_dir: " << FLAGS_log_dir << std::endl;
  LOG_INFO << "GLAGS_minloglevel: " << FLAGS_minloglevel << std::endl;
  LOG_INFO << "GLAGS_alsologtostderr: " << FLAGS_alsologtostderr << std::endl;
  LOG_INFO << "GLAGS_colorlogtostderr: " << FLAGS_colorlogtostderr << std::endl;
}

inline void StopLogging() { google::ShutdownGoogleLogging(); }

