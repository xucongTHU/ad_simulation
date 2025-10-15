// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#pragma once

#include "adaptor_base.h"
#include "adaptor_sensor.h"

#include "cm/base/time.hpp"

static bool g_cm_fource_stop = 0;

namespace stoic::cm {

static void try_exit_safely() {
  if (g_cm_fource_stop) {
    printf("g_cm_fource_stop is: %d\n", g_cm_fource_stop);
    printf("接收到退出信号，已执行完该周期，准备退出...\n");
    usleep(100000);
    printf("接收到退出信号，已执行完该周期，进程已安全退出!!!\n");
    g_cm_fource_stop = 0;
    exit(1);
  } else {
    // printf("g_cm_fource_stop is: %d\n", g_cm_fource_stop);
  }
}

#define HAS_MEMBER(member)\
template<typename T, typename... Args>struct has_member_##member\
{\
    private:\
    template<typename U> static auto Check(int) -> decltype(std::declval<U>().member(std::declval<Args>()...), std::true_type()); \
    template<typename U> static auto Check(...) -> decltype(std::false_type()); \
    public:\
    static const bool value = std::is_same<decltype(Check<T>(0)), std::true_type>::value; \
};

HAS_MEMBER(ByteSizeLong) //has_member_ByteSizeLong

// _Msg is type for caic, for example ::caic_sensor::caic_ins::Odometry
//_RTPtrMsg is protobuf type: proto::common::PString
template <typename _Msg, typename _PStringMsg, bool _Adaptive>
inline void callbackAdaptor(const _PStringMsg& string_msg, void (*callback)(const _Msg&),
                            std::string topic) {
  std::string msg_type = stoic::cm::alg::reflection<_Msg>::fullName();
  bool is_pod = std::is_trivially_copyable<_Msg>::value;
  // _Msg msg;
  std::string topic_name = topic;
  static std::map<std::string, std::shared_ptr<_Msg>> msg_all;
  if (msg_all.find(topic_name) == msg_all.end()) {
    std::shared_ptr<_Msg> msg_sp = std::make_shared<_Msg>();
    msg_all.insert(std::make_pair(topic_name, msg_sp));
  }

  int msg_size;
  bool is_proto = has_member_ByteSizeLong<_Msg>::value;
  
  if(_Adaptive == false || is_proto) {
    using RTType = typename AdaptorTraits<_Msg, false>::RTType;
    std::shared_ptr<RTType> rt_msg = std::make_shared<RTType>();
    msg_size = string_msg->ByteSizeLong();
    {
      stoic::cm::Performance perf(msg_type, msg_size, "recv_parse_from_string",\
         "Warning: This is Proto msg!!!");
      rt_msg->ParseFromString(string_msg->data());
    }
    {
      stoic::cm::Performance perf(msg_type, msg_size, "recv_convert_to_caic",\
        "Warning: This is Proto msg!!!");
      AdaptorTraits<_Msg, false>::convert(rt_msg, msg_all[topic_name].get());
    }
    {
      stoic::cm::Performance perf(msg_type, msg_size, "recv_callback",\
      "Warning: This is Proto msg!!!");
      callback(*(msg_all[topic_name].get()));
    }
  } else {
    if (!is_pod) {
      AERROR << "Msg Type is Not POD, Please Check interface!";
      exit(-1);
    } else {
      msg_size = sizeof(_Msg);
      {
        stoic::cm::Performance perf(msg_type, msg_size, "recv_memcpy", "Msg type is POD");
        memcpy((void*)msg_all[topic_name].get(), (void*)string_msg->data().c_str(), msg_size);
      }
      {
        stoic::cm::Performance perf(msg_type, msg_size, "recv_callback", "Msg type is POD");
        callback(*(msg_all[topic_name].get()));
      }
    }
  }
}

template <typename _Msg, typename _PStringMsg, bool _Adaptive>
inline void callbackAdaptorBoost(const _PStringMsg& string_msg,
                                 const boost::function<void(std::shared_ptr<_Msg>)>& callback,
                                 std::string topic) {
  std::string msg_type = stoic::cm::alg::reflection<_Msg>::fullName();
  bool is_pod = std::is_trivially_copyable<_Msg>::value;
  std::string topic_name = topic;
  static std::map<std::string, std::shared_ptr<_Msg>> msg_all;
  if (msg_all.find(topic_name) == msg_all.end()) {
    std::shared_ptr<_Msg> msg_sp = std::make_shared<_Msg>();
    msg_all.insert(std::make_pair(topic_name, msg_sp));
  }
  int msg_size;
  bool is_proto = has_member_ByteSizeLong<_Msg>::value;
  
  //proto false process
  //proto true process
  //caic false --- exit(-1)
  //caic true ---  
            // not pod exit(-1),如果需要处理，单独特化，如点云
            // pod process
  if(_Adaptive == false || is_proto) {
    using RTType = typename AdaptorTraits<_Msg, false>::RTType;
    std::shared_ptr<RTType> rt_msg = std::make_shared<RTType>();
    msg_size = string_msg->ByteSizeLong();
    {
      stoic::cm::Performance perf(msg_type, sizeof(_Msg), "recv_parse_from_string",
                                  "Warning: Please Use POD MSg!!!");
      rt_msg->ParseFromString(string_msg->data());
    }
    {
      stoic::cm::Performance perf(msg_type, sizeof(_Msg), "recv_convert_to_caic",
                                  "Warning: Please Use POD MSg!!!");
      AdaptorTraits<_Msg, false>::convert(rt_msg, msg_all[topic_name].get());
    }
    {
      stoic::cm::Performance perf(msg_type, sizeof(_Msg), "recv_callback",
                                  "Warning: Please Use POD MSg!!!");
      callback(msg_all[topic_name]);
    }
  } else {
    if (!is_pod) {
      AERROR << "Msg Type is Not POD, Please Check interface!";
      exit(-1);
    } else {
      int msg_size = sizeof(_Msg);
      {
        stoic::cm::Performance perf(msg_type, msg_size, "recv_memcpy", "Msg type is POD");
        memcpy((void*)(msg_all[topic_name].get()), (void*)string_msg->data().c_str(), msg_size);
      }
      {
        stoic::cm::Performance perf(msg_type, msg_size, "recv_callback", "Msg type is POD");
        callback(msg_all[topic_name]);
      }
    }
  }
}

template <>
inline void callbackAdaptor<::caic_sensor::Image, RTImagePtr, true>(
    const RTImagePtr& rt_msg, void (*callback)(const ::caic_sensor::Image&), std::string topic) {
  std::string topic_name = topic;
  using _Msg = ::caic_sensor::Image;
  std::string msg_type = stoic::cm::alg::reflection<_Msg>::fullName();
  static stoic::cm::Performance perf;
  static bool is_debug = perf.get_perf_flag();
  if(is_debug) {
    AINFO << "[CM_DEBUG] start callbackAdaptor, topic is: " + topic_name ;
  }

  static std::map<std::string, std::shared_ptr<::caic_sensor::Image>> image_all;
  if (image_all.find(topic_name) == image_all.end()) {
    std::shared_ptr<::caic_sensor::Image> msg_sp = std::make_shared<::caic_sensor::Image>();
    image_all.insert(std::make_pair(topic_name, msg_sp));
  }
  AdaptorTraits<::caic_sensor::Image, true>::convert(rt_msg, image_all[topic_name]);
  // init
  if (tzcHelper_all.find(topic_name) == tzcHelper_all.end()) {
    printf("tzcHelper_all init.\n");
    autoplt::tzc::TZCHelper<stoic::cm::proto::sensor::Image> tzcHelper(topic_name);
    tzcHelper.InitTZC(8);
    tzcHelper_all.insert(std::make_pair(topic_name, tzcHelper));
    if(is_debug) {
      AINFO << "[CM_DEBUG] recv first frame, topic is: " + topic_name;
    }
  }

  uint8_t* msg_addr = nullptr;
  msg_addr = (uint8_t*)tzcHelper_all.find(topic_name)->second.GetDataPtrFromTZC(rt_msg);
  if (msg_addr == nullptr) {
    usleep(3000);
    msg_addr = (uint8_t*)tzcHelper_all.find(topic_name)->second.GetDataPtrFromTZC(rt_msg);
  }
  if (msg_addr == nullptr) {
    AERROR << "msg_addr is nullptr";
  } else {
    if(is_debug) {
      AINFO << "[CM_DEBUG] start callback, topic is: " + topic_name;
    }
    image_all[topic_name]->p_data = msg_addr;
    callback(*(image_all[topic_name].get()));
    tzcHelper_all.find(topic_name)->second.ReleaseReadLock();
    try_exit_safely();
  }
}

template <>
inline void callbackAdaptorBoost<::caic_sensor::Image, RTImagePtr, true>(
    const RTImagePtr& rt_msg,
    const boost::function<void(std::shared_ptr<::caic_sensor::Image>)>& callback,
    std::string topic) {
  std::string topic_name = topic;
  using _Msg = ::caic_sensor::Image;
  std::string msg_type = stoic::cm::alg::reflection<_Msg>::fullName();
  static stoic::cm::Performance perf;
  static bool is_debug = perf.get_perf_flag();
  if(is_debug) {
    AINFO << "[CM_DEBUG] start callbackAdaptor, topic is: " + topic_name ;
  }

  static std::map<std::string, std::shared_ptr<::caic_sensor::Image>> image_all;
  if (image_all.find(topic_name) == image_all.end()) {
    std::shared_ptr<::caic_sensor::Image> msg_sp = std::make_shared<::caic_sensor::Image>();
    image_all.insert(std::make_pair(topic_name, msg_sp));
  }
  AdaptorTraits<::caic_sensor::Image, true>::convert(rt_msg, image_all[topic_name]);
  // init
  if (tzcHelper_all.find(topic_name) == tzcHelper_all.end()) {
    printf("tzcHelper_all init.\n");
    autoplt::tzc::TZCHelper<stoic::cm::proto::sensor::Image> tzcHelper(topic_name);
    tzcHelper.InitTZC(8);
    tzcHelper_all.insert(std::make_pair(topic_name, tzcHelper));
    if(is_debug) {
      AINFO << "[CM_DEBUG] recv first frame, topic is: " + topic_name;
    }
  }
  uint8_t* msg_addr = nullptr;
  msg_addr = (uint8_t*)tzcHelper_all.find(topic_name)->second.GetDataPtrFromTZC(rt_msg);
  if (msg_addr == nullptr) {
    usleep(3000);
    msg_addr = (uint8_t*)tzcHelper_all.find(topic_name)->second.GetDataPtrFromTZC(rt_msg);
  }
  if (msg_addr == nullptr) {
    AERROR << "msg_addr is nullptr";
  } else {
    if(is_debug) {
      AINFO << "[CM_DEBUG] start callback, topic is: " + topic_name;
    }
    image_all[topic_name]->p_data = msg_addr;
    callback(image_all[topic_name]);
    tzcHelper_all.find(topic_name)->second.ReleaseReadLock();
    try_exit_safely();
  }
}

template <>
inline void callbackAdaptor<::caic_sensor::PointCloudTypeArray, RTPStringPtr, true>(
    const RTPStringPtr& string_msg, void (*callback)(const ::caic_sensor::PointCloudTypeArray&),
    std::string topic) {
  std::string topic_name = topic;
  using _Msg = ::caic_sensor::PointCloudTypeArray;
  using RTType = typename AdaptorTraits<_Msg, true>::RTType;
  std::shared_ptr<RTType> rt_msg = std::make_shared<RTType>();
  std::string msg_type = stoic::cm::alg::reflection<_Msg>::fullName();
  int msg_size;

  static std::map<std::string, std::shared_ptr<caic_sensor::PointCloudTypeArray>> pc_all;
  if (pc_all.find(topic_name) == pc_all.end()) {
    std::shared_ptr<caic_sensor::PointCloudTypeArray> msg_sp = std::make_shared<caic_sensor::PointCloudTypeArray>();
    pc_all.insert(std::make_pair(topic_name, msg_sp));
  }
  msg_size = string_msg->ByteSizeLong();
  {
    stoic::cm::Performance perf(msg_type, sizeof(_Msg), "recv_parse_from_string",
                                "Warning: point cloud msg.");
    rt_msg->ParseFromString(string_msg->data());
  }
  {
    stoic::cm::Performance perf(msg_type, sizeof(_Msg), "recv_convert_to_caic",
                                "Warning: point cloud msg.");
    AdaptorTraits<_Msg, true>::convert(rt_msg, pc_all[topic_name]);
  }
  {
    stoic::cm::Performance perf(msg_type, sizeof(_Msg), "recv_callback",
                                "Warning: point cloud msg.");
    callback(*(pc_all[topic_name].get()));
  }
}

template <>
inline void callbackAdaptorBoost<::caic_sensor::PointCloudTypeArray, RTPStringPtr, true>(
    const RTPStringPtr& string_msg,
    const boost::function<void(std::shared_ptr<::caic_sensor::PointCloudTypeArray>)>& callback,
    std::string topic) {
  std::string topic_name = topic;
  using _Msg = ::caic_sensor::PointCloudTypeArray;
  using RTType = typename AdaptorTraits<_Msg, true>::RTType;
  std::shared_ptr<RTType> rt_msg = std::make_shared<RTType>();
  int msg_size;
  std::string msg_type = stoic::cm::alg::reflection<_Msg>::fullName();

  static std::map<std::string, std::shared_ptr<caic_sensor::PointCloudTypeArray>> pc_all;
  if (pc_all.find(topic_name) == pc_all.end()) {
    std::shared_ptr<caic_sensor::PointCloudTypeArray> msg_sp = std::make_shared<caic_sensor::PointCloudTypeArray>();
    pc_all.insert(std::make_pair(topic_name, msg_sp));
  }
  msg_size = string_msg->ByteSizeLong();
  {
    stoic::cm::Performance perf(msg_type, sizeof(_Msg), "recv_parse_from_string",
                                "Warning: point cloud msg.");
    rt_msg->ParseFromString(string_msg->data());
  }
  {
    stoic::cm::Performance perf(msg_type, sizeof(_Msg), "recv_convert_to_caic",
                                "Warning: point cloud msg.");
    AdaptorTraits<_Msg, true>::convert(rt_msg, pc_all[topic_name]);
  }
  {
    stoic::cm::Performance perf(msg_type, sizeof(_Msg), "recv_callback",
                                "Warning: point cloud msg.");
    callback(pc_all[topic_name]);
  }

}

}  // namespace stoic::cm
