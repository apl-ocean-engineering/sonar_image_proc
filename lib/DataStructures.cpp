#include "serdp_common/DataStructures.h"

namespace serdp_common {
  
SonarPoint bearingRange2Cartesian(float bearing, float range) {
  float x = range * sin(bearing);
  float z = range * cos(bearing);

  return SonarPoint(x, z);
}


AbstractSonarData::AbstractSonarData( const std::vector<float> &b,
                   const std::vector<float> &r,
                   const std::vector<uint8_t> &i )
    : bearings(b),
      ranges(r),
      intensities(i)
      {;}

// Cast constructor
AbstractSonarData::AbstractSonarData( const SimplePingResult &ping )
{
  //todo;
}


} // namespace serdp_common
