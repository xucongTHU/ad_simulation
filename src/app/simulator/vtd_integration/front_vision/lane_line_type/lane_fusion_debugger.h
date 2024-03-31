#pragma once

namespace simulator {
namespace vtd_integration {

class LaneFusionDebug {
 public:
  static LaneFusionDebug* Instance() {
    if (instance_ == nullptr) {
      instance_ = new LaneFusionDebug;
    }

    return instance_;
  }

 private:
  LaneFusionDebug() {}

  static LaneFusionDebug* instance_;

};
LaneFusionDebug* LaneFusionDebug::instance_ = nullptr;

}
}
