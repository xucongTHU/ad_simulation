// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#pragma once

#include <memory>

namespace stoic::cm {

struct SubscriberImpl;

class Subscriber {
 public:
  inline Subscriber(Subscriber&&);
  inline ~Subscriber();

 protected:
  friend class NodeHandle;
  inline Subscriber(std::unique_ptr<SubscriberImpl>&&);

 private:
  std::unique_ptr<SubscriberImpl> impl_;
};

}  // namespace stoic::cm
