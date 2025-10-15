// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#pragma once

#include <memory>

namespace stoic::cm {

struct PublisherImpl;

class Publisher {
 public:
  inline Publisher(Publisher&&);
  inline ~Publisher();
  
  template <typename _Msg, bool _Adaptive>
  inline void publish(const _Msg& msg);

  template<typename _Msg, typename Enable = void>
  inline void publish(const _Msg& msg);

 protected:
  friend class NodeHandle;
  inline Publisher(std::unique_ptr<PublisherImpl>&&);

 private:
  std::unique_ptr<PublisherImpl> impl_;
};

}  // namespace stoic::cm
