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


// Abstract class strictly for drawing sonar images
// Designed as a "common type" between SimplePingResults and ROS ImagingSonarMsg
struct AbstractSonarInterface {

  virtual int nBearings() const = 0;
  virtual float bearing( int n ) const = 0;

  virtual int nRanges() const = 0;
  virtual float range( int n ) const = 0;

  virtual uint8_t intensity( int i ) const  = 0;
  virtual uint8_t intensity( int b, int r ) const
    { return intensity( (r * nBearings()) + b ); }

};

struct SimplePingResultInterface : public AbstractSonarInterface {

  SimplePingResultInterface( const SimplePingResult &ping )
    : _ping(ping) {;}

  virtual int nBearings() const { return _ping.bearings().size(); }
  virtual float bearing( int n ) const { return _ping.bearings().at(n); }

  virtual int nRanges() const { return _ping.oculusPing()->nRanges; }
  virtual float range( int n ) const { return _ping.oculusPing()->rangeResolution * n; }

  virtual uint8_t intensity( int i ) const { return _ping.image().at( i % nBearings(), floor(i/nBearings()) ); }

  const SimplePingResult &_ping;
};


SonarPoint bearingRange2Cartesian(float bearing, float range);

} // namespace serdp_common
