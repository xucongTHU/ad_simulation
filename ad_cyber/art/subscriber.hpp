// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#pragma once

#include "ad_interface.h"
#include "cm/subscriber.h"
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "tzc/tzc_helper.h"

namespace stoic::cm {

using RTPString = stoic::cm::proto::common::PString;
using RTImage = stoic::cm::proto::sensor::Image;

struct SubscriberImpl {
  using readerNull = nullptr_t;
  using readerRTPString = std::shared_ptr<apollo::cyber::Reader<RTPString>>;
  using readerRTImage = std::shared_ptr<apollo::cyber::Reader<RTImage>>;

  SubscriberImpl(readerNull&& sub) : sub_(std::forward<readerNull>(sub)) {}

  SubscriberImpl(readerRTPString&& sub) : sub_string(std::forward<readerRTPString>(sub)) {}
  SubscriberImpl(readerRTImage&& sub) : sub_image(std::forward<readerRTImage>(sub)) {}
  readerNull sub_;
  readerRTPString sub_string;
  readerRTImage sub_image;
};

Subscriber::Subscriber(Subscriber&&) = default;

Subscriber::Subscriber(std::unique_ptr<SubscriberImpl>&& impl)
    : impl_(std::forward<std::unique_ptr<SubscriberImpl>>(impl)) {}

Subscriber::~Subscriber() = default;

}  // namespace stoic::cm
