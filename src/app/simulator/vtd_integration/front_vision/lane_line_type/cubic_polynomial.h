#pragma once

#include "caic_interface/caic_perception.h"

namespace simulator {
namespace vtd_integration {

// y = c0 + c1 * t + c2 * t^2 + c3 * t^3
class cubic_polynomial {
 public:
  cubic_polynomial(double c0, double c1, double c2, double c3)
      : c0_(c0), c1_(c1), c2_(c2), c3_(c3) {}

 public:
  double Evaluate(const double& t, const unsigned char& order = 0) const;

  double Heading(const double& t) const;

  double Kappa(const double& t) const;

  double c0() const { return c0_; }
  double c1() const { return c1_; }
  double c2() const { return c2_; }
  double c3() const { return c3_; }

  double& MutableC0() { return c0_; }
  double& MutableC1() { return c1_; }
  double& MutableC2() { return c2_; }
  double& MutableC3() { return c3_; }

 public:
 private:
  double c0_ = 0.0;
  double c1_ = 0.0;
  double c2_ = 0.0;
  double c3_ = 0.0;
};

}
}
