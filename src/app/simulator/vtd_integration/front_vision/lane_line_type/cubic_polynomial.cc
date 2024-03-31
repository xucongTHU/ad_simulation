#include "simulator/vtd_integration/front_vision/lane_line_type/cubic_polynomial.h"
#include <math.h>

namespace simulator {
namespace vtd_integration {

constexpr double kDefaultLaneWidth = 3.5;

double cubic_polynomial::Evaluate(
    const double& t, const unsigned char& order) const {
  if (order == 0) {
    return (c0_ + t * (c1_ + t * (c2_ + t * c3_ )));
  } else if (order == 1) {
    return (c1_ + t * (2 * c2_ + 3 * t * c3_ ));
  } else if (order == 2) {
    return (2 * c2_ + 6 * t * c3_ );
  } else if (order == 3) {
    return (6 * c3_);
  } else {
    return 0.0;
  }
}

double cubic_polynomial::Heading(const double& t) const {
  return atan2(Evaluate(t, 1), 1.0);
}

double cubic_polynomial::Kappa(const double& t) const {
  const double ddy = Evaluate(t, 2);
  const double dy = Evaluate(t, 1);
  return ddy / (1 + dy * dy) / sqrt(1 + dy * dy);
}
};
}



