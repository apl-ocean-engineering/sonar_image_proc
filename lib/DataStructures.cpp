#include <limits>

#include "draw_sonar/DataStructures.h"

namespace draw_sonar {

SonarPoint bearingRange2Cartesian(float bearing, float range) {
  float x = range * sin(bearing);
  float z = range * cos(bearing);

  return SonarPoint(x, z);
}


float AbstractSonarInterface::minRange() const {
  if( _minRange > 0.0 ) return _minRange;

  _minRange = std::numeric_limits<float>::max();

  // Ah, wish I had iterators
  for( int i = 0; i < nRanges(); i++ ) {
    if( range(i) < _minRange ) _minRange = range(i);
  }

  return _minRange;
}


float AbstractSonarInterface::maxRange() const {
  if( _maxRange > 0.0 ) return _maxRange;

  for( int i = 0; i < nRanges(); i++ ) {
    if( range(i) > _maxRange ) _maxRange = range(i);
  }

  return _maxRange;
}

} // namespace serdp_common
