#pragma once

#include <stdint.h>
#include <math.h>
#include <vector>

namespace draw_sonar {

struct SonarPoint {
  SonarPoint(float _x, float _z) : x(_x), z(_z) { ; }
  float x;
  float z;
};

SonarPoint bearingRange2Cartesian(float bearing, float range);

} // namespace serdp_common
