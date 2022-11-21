// Copyright 2021 University of Washington Applied Physics Laboratory
//

#pragma once

#include <math.h>
#include <stdint.h>

#include <vector>

namespace sonar_image_proc {

// \todo.  Are these used anymore?
struct SonarPoint {
  SonarPoint(float _x, float _z) : x(_x), z(_z) { ; }
  float x;
  float z;
};

SonarPoint bearingRange2Cartesian(float bearing, float range);

}  // namespace sonar_image_proc
