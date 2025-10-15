// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#pragma once

#include <signal.h>
#include <time.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

#include <map>
#include <unordered_map>

#if defined MW_ART_IPC || MW_ART_ORIN
#include "cyber/time/clock.h"
#endif

namespace stoic::cm {

static std::unordered_map<std::string, uint32_t> topic_num_;

// time since epoch
using TimeUS = int64_t;  // microsecond
using TimeMS = int64_t;  // millisecond
using TimeSEC = double;  // second
using TimeDelta = int32_t;

using Time = TimeUS;

static constexpr int32_t kMilliPerSecond = 1'000'000;
static constexpr int32_t kMicroPerSecond = 1'000;

struct Timer {
  static TimeUS now_us() {
    std::chrono::microseconds microsec = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch());
    return TimeUS(microsec.count());
  }

  static TimeMS now_ms() {
    std::chrono::milliseconds millisec = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch());
    return TimeMS(millisec.count());
  }

  static TimeSEC now_sec() {
    std::chrono::seconds sec = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch());
    return TimeSEC(sec.count());
  }

  static void sleep_us(const TimeUS& duration) {
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
  }

  static void sleep_ms(const TimeMS& duration) {
    std::this_thread::sleep_for(std::chrono::milliseconds(duration));
  }

  static Time now() {
    Time now;
#if defined MW_ROS_IPC || MW_ROS_ORIN
    now = ros::Time::now().toSec() * 1000000;
#endif

#ifdef MW_AP
    struct timespec tp {};
    (void)clock_gettime(CLOCK_REALTIME, &tp);
    now = static_cast<Time>(tp.tv_sec * 1000000 + tp.tv_nsec * 1e-3);
#endif

#ifdef MW_RT
    static autoplt::simtime::SimClockSource sim;
    auto cur_sim_time = sim.GetSimNow();
    now = static_cast<Time>(cur_sim_time * 1e-3);
#endif

#if defined MW_ART_IPC || MW_ART_ORIN
    now = (TimeUS)apollo::cyber::Clock::now_us();
#endif

    return now;
  }
};


#if defined MW_ROS_IPC || MW_ROS_ORIN
typedef ::ros::Rate Rate;
#elif defined MW_ART_IPC || MW_ART_ORIN
typedef apollo::cyber::RateV2 Rate;
#else
class Rate {
 public:
  Rate(const size_t& rate) : rate_(rate) {
    gap_ = static_cast<double>(kMilliPerSecond) / rate;
    expect_time_ = Timer::now_us() + gap_;
  }

  void sleep() {
    TimeUS duration = expect_time_ - Timer::now_us();
    if (duration > 0) {
      Timer::sleep_us(duration);
    }
    expect_time_ = Timer::now_us() + gap_;
  }

 private:
  size_t rate_;
  TimeUS gap_;
  TimeUS expect_time_;
};
#endif

class TicToc {
 public:
  TicToc() { tic(); }

  inline void tic() { start_ = Timer::now_us(); }

  inline TimeDelta toc() { return Timer::now_us() - start_; }

 private:
  Time start_;
};

class Performance {
 public:
  Performance() {
    is_printf_ = 1;
    std::string cyber_perf_path = std::string(getenv("CM_PERF_FLAG"));
    int cyber_perf_path_i = std::stoi(cyber_perf_path);
    if ((bool)std::stoi(cyber_perf_path)) {
      cm_perf_flag_ = 1;
    } else {
      cm_perf_flag_ = 0;
    }
  }

  Performance(const std::string& str) {
    description_ << str;
    is_printf_ = 1;
  }

  Performance(const std::string& str, const bool& is_printf) {
    description_ << str;
    is_printf_ = is_printf;
  }

  Performance(const std::string& msg_name, const int& msg_size, const std::string fun_name,
              std::string info = " ") {
    description_ << msg_name;
    is_printf_ = 0;
    fun_name_ = fun_name;
    msg_size_ = msg_size;
    info_ = info;
    std::string cyber_perf_path = std::string(getenv("CM_PERF_FLAG"));
    int cyber_perf_path_i = std::stoi(cyber_perf_path);
    if ((bool)std::stoi(cyber_perf_path)) {
      cm_perf_flag_ = 1;
    } else {
      cm_perf_flag_ = 0;
    }

    // init map
    key_value_ = fun_name_ + "|" + msg_name;
    if (topic_num_[key_value_] > 1) {
      ;
    } else {
      if (topic_num_.find(key_value_) == topic_num_.end()) {
        topic_num_.insert(std::make_pair(key_value_, 0));
      }
    }
  }

  inline void tic() { timer_.tic(); }

  inline void toc_perf() {
    TimeDelta duration = timer_.toc();
    topic_num_[key_value_]++;  //当前topic 的累计数量
    if (cm_perf_flag_) {
      if (topic_num_[key_value_] % 200 == 0) {
        AINFO << "|" + std::string(getenv("CM_VERSION")) +"|" << fun_name_ << "|" << description_.str().c_str() << "|"
              << msg_size_ / 1024.0f << "|k-Bytes, time cost:|" << duration / 1'000.0 << "|ms|"
              << info_ << "|";
      }
    }
  }

  inline void toc() {
    TimeDelta duration = timer_.toc();
    std::streamsize ss = std::cout.precision();
    if (is_printf_) {
      std::cout << std::setprecision(2);  // PRQA S 4400 # performance used.
      std::cout << description_.str() << " time cost: " << std::fixed << duration / 1'000.0
                << " ms"  // PRQA S 4400 # performance used.
                << std::endl;
      std::cout << std::setprecision(ss);
    }
  }

  bool get_perf_flag() {
    return cm_perf_flag_;
  }

  ~Performance() {
    if (is_printf_) {
      toc();
    } else {
      toc_perf();
    }
  }

 private:
  std::stringstream description_;
  TicToc timer_;

  bool is_printf_;
  bool cm_perf_flag_ = 0;
  int msg_size_;
  std::string fun_name_;
  std::string path_string_;
  std::string key_value_;
  std::string info_;
};

using PERF = Performance;

template <typename T>
inline Time getAverageTime(T& frame) {
  Time average = 0;
  for (auto& ele : frame->msg_mp_) {
    average += ele.second.header.stamp;
  }
  return average / (int64_t)frame->msg_mp_.size();
};
}  // namespace stoic::cm
