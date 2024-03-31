// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#pragma once

#include "cm/subscriber.h"

#include <ros/ros.h>

namespace stoic::cm {

struct SubscriberImpl {
  SubscriberImpl(ros::Subscriber&& subscriber) : sub(std::forward<ros::Subscriber>(subscriber)) {}

  ros::Subscriber sub;
};

Subscriber::Subscriber(Subscriber&&) = default;

Subscriber::Subscriber(std::unique_ptr<SubscriberImpl>&& impl)
    : impl_(std::forward<std::unique_ptr<SubscriberImpl>>(impl)) {}

Subscriber::~Subscriber() = default;

}  // namespace stoic::cm
