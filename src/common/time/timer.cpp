// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#if defined MW_ROS_IPC || MW_ROS_ORIN
#include <ros/time.h>
#endif

#include <time.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include "common/time/timer.h"
#include "common/log/Logger.h"

namespace stoic {

// String to identify log entries originating from this file.
static const char* LOG_TAG = "time";

Performance::Performance(const std::string& str) {
  description_ << str;
  is_printf_ = 1;
}

Performance::Performance(const std::string& str, const bool& is_printf) {
  description_ << str;
  is_printf_ = is_printf;
}

}  // namespace stoic
