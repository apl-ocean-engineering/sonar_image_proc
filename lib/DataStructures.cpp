#include "draw_sonar/DataStructures.h"

namespace draw_sonar {

SonarPoint bearingRange2Cartesian(float bearing, float range) {
  float x = range * sin(bearing);
  float z = range * cos(bearing);

  return SonarPoint(x, z);
}


} // namespace serdp_common
