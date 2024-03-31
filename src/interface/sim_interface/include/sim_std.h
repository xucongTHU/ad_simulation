#ifndef SIM_INTERFACE_SIM_STD_H
#define SIM_INTERFACE_SIM_STD_H

#include <array>
#include <stdint.h>
#include <string>
#include <vector>

#include "sim_pod_cfg.h"

namespace sim_std {

struct Header {
  uint32_t seq;
  int64_t stamp;
  std::string frame_id;
};

struct HeaderPOD {
  uint32_t seq;
  int64_t stamp;
  char frame_id[STD_FRAME_ID_SIZE];
  char module_name[STD_MODULE_NAME_SIZE];
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
 * \brief  point 2d
 *\point position in two-dimensional coordinates(m)
 * \unit{m}
 */
struct Point2d {
  double x;
  double y;
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

struct MessageBase {
  void ParseFromString(const std::string&) {}
  void SerializeToString(std::string*) {}
};


} // namespace caic_std

#endif
