
#pragma once

#include "third_party/smartsim/math/vec2d.h"
#include "third_party/smartsim/proto/geometry/geometry.pb.h"
#include "third_party/smartsim/proto/geometry/pnc_point.pb.h"

namespace smartsim {
namespace common {

using smartsim::geometry::SLPoint;
using smartsim::geometry::PointENU;
using smartsim::geometry::PathPoint;
using smartsim::geometry::SpeedPoint;

class PointFactory {
 public:
  template <typename XY>
  static inline smartsim::math::Vec2d ToVec2d(const XY& xy) {
    return smartsim::math::Vec2d(xy.x(), xy.y());
  }

  static inline SLPoint ToSLPoint(const double s, const double l) {
    SLPoint sl;
    sl.set_s(s);
    sl.set_l(l);
    return sl;
  }

  static inline PointENU ToPointENU(const double x, const double y,
                                    const double z = 0) {
    PointENU point_enu;
    point_enu.set_x(x);
    point_enu.set_y(y);
    point_enu.set_z(z);
    return point_enu;
  }

  template <typename XYZ>
  static inline PointENU ToPointENU(const XYZ& xyz) {
    return ToPointENU(xyz.x(), xyz.y(), xyz.z());
  }

  static inline SpeedPoint ToSpeedPoint(const double s, const double t,
                                        const double v = 0, const double a = 0,
                                        const double da = 0) {
    SpeedPoint speed_point;
    speed_point.set_s(s);
    speed_point.set_t(t);
    speed_point.set_v(v);
    speed_point.set_a(a);
    speed_point.set_da(da);
    return speed_point;
  }

  static inline PathPoint ToPathPoint(const double x, const double y,
                                      const double z = 0, const double s = 0,
                                      const double theta = 0,
                                      const double kappa = 0,
                                      const double dkappa = 0,
                                      const double ddkappa = 0) {
    PathPoint path_point;
    path_point.set_x(x);
    path_point.set_y(y);
    path_point.set_z(z);
    path_point.set_s(s);
    path_point.set_theta(theta);
    path_point.set_kappa(kappa);
    path_point.set_dkappa(dkappa);
    path_point.set_ddkappa(ddkappa);
    return path_point;
  }
};

}  // namespace common
}  // namespace smartsim
