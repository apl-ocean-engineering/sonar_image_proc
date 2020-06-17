#pragma once

#include <stdint.h>
#include <math.h>
#include <vector>

#include "liboculus/SimplePingResult.h"

namespace serdp_common {

using namespace liboculus;

struct SonarPoint {
  SonarPoint(float _x, float _z) : x(_x), z(_z) { ; }
  float x;
  float z;
};

struct SonarData {
  SonarData() : timestamp(0.0), frequency(-1.0) { ; }
  unsigned int nBearings;
  unsigned int nRanges;
  float timestamp;
  float frequency;
  std::vector<float> bearings;
  std::vector<float> ranges;
  std::vector<float> intensities;
};


// Abstract class strictly for drawing sonar images
// Designed as a "common type" between SimplePingResults and ROS ImagingSonarMsg
struct AbstractSonarData {

  AbstractSonarData( const std::vector<float> &bearings,
                     const std::vector<float> &ranges,
                     const std::vector<uint8_t> &intensities );

  // Cast constructor
  AbstractSonarData( const SimplePingResult &ping );

  std::vector<float> bearings;
  std::vector<float> ranges;
  std::vector<uint8_t> intensities;
};


SonarPoint bearingRange2Cartesian(float bearing, float range);
} // namespace serdp_common
