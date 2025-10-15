#pragma once
#include <utility>
#include <assert.h>
#include "third_party/smartsim/common/config_gflags.h"

#define PLANNING_UNIQUE_LOCK_MULTITHREAD(mutex_type)                         \
  std::unique_ptr<std::unique_lock<std::mutex>> lock_ptr = nullptr; \
  if (FLAGS_multithread_run) {                                      \
    lock_ptr.reset(new std::unique_lock<std::mutex>(mutex_type));   \
  }

template <class Collection>
const typename Collection::value_type::second_type&
FindOrDie(const Collection& collection,
          const typename Collection::value_type::first_type& key) {
  typename Collection::const_iterator it = collection.find(key);
  assert(it != collection.end());
  return it->second;
}
  
namespace smartsim {
namespace common {
  
template <typename ProtoA, typename ProtoB>
bool IsProtoEqual(const ProtoA& a, const ProtoB& b) {
  return a.GetTypeName() == b.GetTypeName() &&
         a.SerializeAsString() == b.SerializeAsString();
}

template <typename U, typename V>
bool SamePointXY(const U& u, const V& v) {
  static constexpr double kMathEpsilonSqr = 1e-8 * 1e-8;
  return (u.x() - v.x()) * (u.x() - v.x()) < kMathEpsilonSqr &&
         (u.y() - v.y()) * (u.y() - v.y()) < kMathEpsilonSqr;
}

template <typename T>
void uniform_slice(const T start, const T end, uint32_t num,
                   std::vector<T>* sliced) {
  if (!sliced || num == 0) {
    return;
  }
  const T delta = (end - start) / num;
  sliced->resize(num + 1);
  T s = start;
  for (uint32_t i = 0; i < num; ++i, s += delta) {
    sliced->at(i) = s;
  }
  sliced->at(num) = end;
}

}  // namespace common
}  // namespace smartsim
