// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#pragma once

#include "cm/art/msgs/adaptor.h"
#include "cm/publisher.h"
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "tzc/ARTNode.h"
#include "tzc/tzc_helper.h"

#include "cm/base/time.hpp"

namespace stoic::cm {

using RTString = stoic::cm::proto::common::PString;
using RTImage = stoic::cm::proto::sensor::Image;

struct PublisherImpl {
  using PubCommonString = std::shared_ptr<apollo::cyber::Writer<RTString>>;
  using PubSensorImage = std::shared_ptr<apollo::cyber::Writer<RTImage>>;

  PublisherImpl(PubSensorImage&& pub) : pub_sensor_image(std::forward<PubSensorImage>(pub)) {
    printf("PublisherImpl GetChannelNameis: %s\n", pub_sensor_image->GetChannelName().c_str());
    p_tzcHelper_ =
        std::make_shared<autoplt::tzc::TZCHelper<RTImage>>(pub_sensor_image->GetChannelName());

    auto tzcHInitSucc = p_tzcHelper_->InitTZC(16);  // TODO
    if (!tzcHInitSucc) {
      AERROR << "[Example tzc talker] TZC talker Init failed.";
    } else {
      printf("CM InitTZC for PubSensorImage ok.\n");
    }
  }

  PublisherImpl(PubCommonString&& pub) : pub_common_string(std::forward<PubCommonString>(pub)) {}

  PubSensorImage pub_sensor_image;
  PubCommonString pub_common_string;

  // shm for image.
  std::shared_ptr<autoplt::tzc::TZCHelper<RTImage>> p_tzcHelper_;
};

Publisher::Publisher(std::unique_ptr<PublisherImpl>&& impl)
    : impl_(std::forward<std::unique_ptr<PublisherImpl>>(impl)) {}

Publisher::Publisher(Publisher&&) = default;

Publisher::~Publisher() = default;

template <typename _Msg, bool _Adaptive = true>
inline void Publisher::publish(const _Msg& msg) {
  using T = _Msg;
  std::string msg_type = stoic::cm::alg::reflection<_Msg>::fullName();
  bool is_pod = std::is_trivially_copyable<T>::value;
  stoic::cm::proto::common::PString rtstring;
  //for proto msg.
  if(_Adaptive == false) {
    typename AdaptorTraits<T, _Adaptive>::RTType rt_msg;
    std::string s;
    {
      stoic::cm::Performance perf(msg_type, 0, "send_convert_to_proto",
                                  "Warning: This is Proto msg!!!");
      AdaptorTraits<T, _Adaptive>::convert(msg, rt_msg);
    }
    {
      stoic::cm::Performance perf(msg_type, 0, "send_serialize_to_string",
                                  "Warning: This is Proto msg!!!");
      rt_msg.SerializeToString(&s);
    }
    {
      stoic::cm::Performance perf(msg_type, s.length(), "send_pub_write",
                                  "Warning: This is Proto msg!!!");
      rtstring.set_data(std::move(s));
      impl_->pub_common_string->Write(rtstring);
    }
  } else {
    //for struct
    if (!is_pod) {
      AERROR << "Publish Msg Type is Not POD type!!!!!";
      exit(-1);
    } else {
      int msg_size = sizeof(T);
      {
        stoic::cm::Performance perf(msg_type, msg_size, "send_set_proto_data", "Msg type is POD");
        rtstring.set_data((u_char*)&msg, msg_size);
      }
      {
        stoic::cm::Performance perf(msg_type, msg_size, "send_pub_write", "Msg type is POD");
        impl_->pub_common_string->Write(rtstring);
      }
    }
  }
}

template <>
inline void Publisher::publish<::caic_sensor::Image, true>(const ::caic_sensor::Image& msg) {
  using T = ::caic_sensor::Image;
  using RTType = AdaptorTraits<T, true>::RTType;
  std::shared_ptr<RTType> p_rt_msg = std::make_shared<RTType>();
  AdaptorTraits<T, true>::convert(msg, p_rt_msg);
  // shm
  static apollo::cyber::transport::WritableBlock wb;
  impl_->p_tzcHelper_->GetSHMBlockToWrite(msg.size, wb, p_rt_msg);
  memcpy(wb.buf, msg.p_data, msg.size);
  impl_->pub_sensor_image->Write(p_rt_msg);
  impl_->p_tzcHelper_->ReleaseWriteLock(wb);
  try_exit_safely();
}

template <>
inline void Publisher::publish<::caic_sensor::PointCloudTypeArray, true>(
    const ::caic_sensor::PointCloudTypeArray& msg) {
  using T = ::caic_sensor::PointCloudTypeArray;
  using RTType = AdaptorTraits<T, true>::RTType;  // proto type
  std::string msg_type = stoic::cm::alg::reflection<T>::fullName();
  std::shared_ptr<RTType> rt_msg = std::make_shared<RTType>();
  stoic::cm::proto::common::PString rtstring;
  int msg_size = sizeof(T);
  std::string s;
  {
    stoic::cm::Performance perf(msg_type, msg_size, "send_convert_to_proto",
                            "Warning: point cloud msg.");
    AdaptorTraits<T, true>::convert(msg, rt_msg);
  }
  
  {
    stoic::cm::Performance perf(msg_type, msg_size, "send_SerializeToString",
                            "Warning: point cloud msg.");
    rt_msg->SerializeToString(&s);
  }

  {
    stoic::cm::Performance perf(msg_type, msg_size, "send_Write",
                            "Warning: point cloud msg.");
    rtstring.set_data(std::move(s));
    impl_->pub_common_string->Write(rtstring);
  }

}

}  // namespace stoic::cm
