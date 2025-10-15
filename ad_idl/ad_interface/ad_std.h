#ifndef ad_INTERFACE_ad_STD_H
#define ad_INTERFACE_ad_STD_H

#include <array>
#include <stdint.h>
#include <string>
#include <vector>

#include "ad_pod_cfg.h"
namespace ad_std {

struct Header {
  uint32_t seq;
  int64_t stamp;
  std::string frame_id;
  std::string module_name;
};
struct HeaderPOD {
  uint32_t seq;
  int64_t stamp;
  char frame_id[STD_FRAME_ID_SIZE];
  char module_name[STD_MODULE_NAME_SIZE];
};

struct MessageBase {
  void ParseFromString(const std::string&) {}
  void SerializeToString(std::string*) {}
};

/*!
 * \brief  WGS84Point3i
 *\point position in three-dimensional coordinates(LLH)
 * \unit The unit of longitude and latitude is 10^-7 degree ,The unit of Height is cm
 */
 struct WGS84Point3i {
   uint32_t longitude;
   uint32_t latitude;
   uint32_t height;
 };

/*!
* \brief  WGS84Point3d
*\point position in three-dimensional coordinates(LLH)
* \unit The unit of longitude and latitude is degree ,The unit of Height is m
*/
struct WGS84Point3d {
  double longitude;
  double latitude;
  double height;
};

/*!
 * \brief  point 3i
 *\point position in three-dimensional coordinates(cm)
 * \unit{cm}
 */
struct Point3i {
  uint32_t x;
  uint32_t y;
  uint32_t z;
};

/*!
 * \brief  point 3f
 *\point position in three-dimensional coordinates(m)
 * \unit{m}
 */
struct Point3f {
  float x;
  float y;
  float z;
};

/*!
 * \brief  point 3d
 *\point position in three-dimensional coordinates(m)
 * \unit{m}
 */
struct Point3d {
  double x;
  double y;
  double z;
};

/*!
 * \brief  point 2i
 *\point position in two-dimensional coordinates(cm)
 * \unit{cm}
 */
struct Point2i {
  uint32_t x;
  uint32_t y;
};

/*!
 * \brief  point 2d
 *\point position in two-dimensional coordinates(m)
 * \unit{m}
 */
struct Point2d {
  double x;
  double y;
};

/*!
 * \brief  point 2f
 *\point position in two-dimensional coordinates(m)
 * \unit{m}
 */
struct Point2f {
  float x;
  float y;
};

/*!
 * \brief  vector 3d
 *\vector in three-dimensional coordinates
 */
struct Vector3d {
  double x;
  double y;
  double z;
};

/*!
 * \brief  vector 2d
 *\vector in two-dimensional coordinates
 */
struct Vector2d {
  double x;
  double y;
};

/*!
 * \brief  CovarianceMatrix
 *\Customed covariance matrix of different dimensions and data types
 */
template <typename T, size_t N> struct CovarianceMatrix {
  std::array<T, N * N> data;
};

template <typename T, size_t N> struct ImageArray { std::array<T, N> data; };

/*!
 * \brief  Polygon2d
 *\A Polygon2D is defined by a set of points. Each point is connected to the
 *next, with the final point being connected to the first, resulting in a closed
 *polygon.
 */
using Polygon2d = std::vector<Vector2d>;

/**
 * @brief Quaternion
 *  the quotient of two directed lines in a three-dimensional space
 */
struct Quaternion {
  double x;
  double y;
  double z;
  double w;
};
template<typename T>
struct PathPoint {
  /// coordinates
  T x;
  T y;
  T z;
  /// direction on the x-y plane
  T theta;
  /// curvature on the x-y planning
  T kappa;
  /// accumulated distance from beginning of the path
  T s;
  /// derivative of kappa w.r.t s.
  T dkappa;
  /// derivative of derivative of kappa w.r.t s.
  T ddkappa;
  /// The lane ID where the path point is on
  char lane_id[STD_LANE_ID_SIZE];
  /// derivative of x and y w.r.t parametric parameter t in
  /// CosThetareferenceline
  T x_derivative;
  T y_derivative;
};

struct TrajectoryPoint {
  /// path point
  PathPoint<float> path_point;
  /// linear velocity in [m/s]
  float v;
  /// linear acceleration
  float a;
  /// relative time from beginning of the trajectory
  int64_t relative_time_us;
  /// longitudinal jerk
  float da;
  /// The angle between vehicle front wheel and vehicle longitudinal axis
  float steer;
};
struct VehicleSignal {
  enum struct TurnSignal {
    TURN_NONE,
    TURN_LEFT,
    TURN_RIGHT,
  };

  TurnSignal turn_signal = TurnSignal::TURN_NONE;
  /// lights enable command
  bool high_beam = false;
  bool low_beam = false;
  bool horn = false;
  bool emergency_light = false;
};

/*!
 * \brief  Gear info
 */
enum struct GearState : uint8_t {
  NEUTRAL = 0,
  DRIVE,
  REVERSE,
  PARKING,
  LOW,
  INVALID,
  NONE,
};

/**
 * @brief This represents a pose in free space with uncertainty
 * The orientation parameters use a fixed-axis representation. In order, the
 * parameters are: (x, y, z, rotation about X axis, rotation about Y axis,
 * rotation about Z axis)
 */
struct PoseWithCovariance {
  uint8_t available;     ///< binary,1 for available,0 for unavailable
  Point3d position;      ///< the position of a point in free space
  Quaternion quaternion; ///< an orientation in free space in quaternion form
  CovarianceMatrix<double, 6>
      covariance; ///< Row-major representation of the 6x6 covariance matrix

  enum : uint8_t {
    POSITION = 1 << 0,
    ORIENTATION = 1 << 1,
    COVARINACE = 1 << 2,
  };
};

struct OrientationWithCovariance {
  uint8_t available; ///< binary,1 for available,0 for unavailable
  ad_std::Quaternion
      quaternion; ///< the rotation from imu frame to the world frame

  ad_std::CovarianceMatrix<double, 3>
      quaternion_covariance; ///< Row major about x, y, z axes
  enum : uint8_t {
    VALUE = 1 << 0,
    QUATERNION_COVARIANCE = 1 << 1,
  };
};

struct LinearVelocityWithCovariance {
  uint8_t available;        ///< binary,1 for available,0 for unavailable
  Vector3d linear_velocity; ///< linear velocity, unit: m/s
  CovarianceMatrix<double, 3>
      linear_velocity_covariance; ///< Row major about x, y, z axes

  enum : uint8_t {
    VALUE = 1 << 0,
    LINEAR_VELOCITY_COVARIANCE = 1 << 1,
  };
};

struct LinearAccelerationWithCovariance {
  uint8_t available;            ///< binary,1 for available,0 for unavailable
  Vector3d linear_acceleration; ///< linear acceleration, unit: m/s^2
  CovarianceMatrix<double, 3>
      linear_acceleration_covariance; ///< Row major about x, y, z axes

  enum : uint8_t {
    VALUE = 1 << 0,
    LINEAR_ACCELERATION_COVARIANCE = 1 << 1,
  };
};

struct AngularVelocityWithCovariance {
  uint8_t available;         ///< binary,1 for available,0 for unavailable
  Vector3d angular_velocity; ///< angular velocity, unit: rad/s
  CovarianceMatrix<double, 3>
      angular_velocity_covariance; ///< Row major about x, y, z axes

  enum : uint8_t {
    VALUE = 1 << 0,
    ANGULAR_VELOCITY_COVARIANCE = 1 << 1,
  };
};

/**
 * @brief
 * This expresses velocity in free space broken into its linear and angular
 * parts Row-major representation of the 6x6 covariance matrix The orientation
 * parameters use a fixed-axis representation. In order, the parameters are: (x,
 * y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
 */

struct TwistWithCovariance {
  uint8_t available;          ///< binary,1 for available,0 for unavailable
  ad_std::Vector3d linear;  ///< linear velocity, unit: m/s^2
  ad_std::Vector3d angular; ///< angular velocity, unit: rad/s
  ad_std::CovarianceMatrix<double, 6>
      covariance; ///< Row-major representation of the 6x6 covariance matrix

  enum : uint8_t {
    LINEAR = 1 << 0,
    ANGULAR = 1 << 1,
    COVARINACE = 1 << 2,
  };
};

enum class ImageEncoding : uint8_t {
  YUYV = 0,
  UYVY,
  RGB,
  BGR,
  NV12,
  YUV420_NV12,
  YUV420_NV21,
  YUV422_UYVY,
  YUV422_YUYV,
  RGB888_PLANAR,
  BGR888_PLANAR,
  RGB888_PACKAGE,
  BGR888_PACKAGE,
  GRAY,
};

template <size_t W_MAX, size_t H_MAX, size_t C_MAX>
struct Image : public ad_std::MessageBase {
  ad_std::HeaderPOD header;
  ImageEncoding encoding;
  uint32_t width;          ///< actual image columns
  uint32_t height;         ///< actual image rows
  uint32_t size;           ///< actual used size
  uint8_t data[W_MAX * H_MAX * C_MAX]; //max size is W_MAX * H_MAX * C_MAX
};

} // namespace ad_std

#endif
