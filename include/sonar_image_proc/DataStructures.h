#pragma once

#include <stdint.h>
#include <math.h>
#include <vector>

namespace sonar_image_proc {

struct SonarPoint {
  SonarPoint(float _x, float _z) : x(_x), z(_z) { ; }
  float x;
  float z;
};

SonarPoint bearingRange2Cartesian(float bearing, float range);

}
