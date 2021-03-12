#pragma once

#include <stdint.h>
#include <math.h>
#include <vector>

namespace sonar_image_proc {

// Abstract class strictly for drawing sonar images
// Designed as a "common type" between SimplePingResults and ROS ImagingSonarMsg
struct AbstractSonarInterface {

  AbstractSonarInterface()
    : _minRange(-1), _maxRange(-1)
    {;}

  virtual int nBearings() const = 0;

  // bearing is in __radians__
  virtual float bearing( int n ) const = 0;

  virtual int nRanges() const = 0;
  virtual float range( int n ) const = 0;

  float minRange() const {
    if( _minRange > 0.0 ) return _minRange;

    _minRange = std::numeric_limits<float>::max();

    // Ah, wish I had iterators
    for( int i = 0; i < nRanges(); i++ ) {
      if( range(i) < _minRange ) _minRange = range(i);
    }

    return _minRange;
  }

  float maxRange() const {
    if( _maxRange > 0.0 ) return _maxRange;

    for( int i = 0; i < nRanges(); i++ ) {
      if( range(i) > _maxRange ) _maxRange = range(i);
    }

    return _maxRange;
  }

  virtual uint8_t intensity( int i ) const  = 0;

  // Data _must_ be bearing-major
  uint8_t intensity( int b, int r ) const
    { return intensity( (r * nBearings()) + b ); }


private:

  mutable float _minRange, _maxRange;

};

}
