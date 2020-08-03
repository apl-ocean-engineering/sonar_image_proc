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


// Abstract class strictly for drawing sonar images
// Designed as a "common type" between SimplePingResults and ROS ImagingSonarMsg
struct AbstractSonarInterface {

  virtual int nBearings() const = 0;

  // bearing is in __radians__
  virtual float bearing( int n ) const = 0;

  virtual int nRanges() const = 0;
  virtual float range( int n ) const = 0;

  virtual uint8_t intensity( int i ) const  = 0;

  // Data _must_ be bearing-major
  virtual uint8_t intensity( int b, int r ) const
    { return intensity( (r * nBearings()) + b ); }

};


SonarPoint bearingRange2Cartesian(float bearing, float range);

} // namespace serdp_common
