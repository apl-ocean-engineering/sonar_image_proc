#include "serdp_common/PingDecoder.h"

namespace serdp_common {

PingDecoder::SonarData
PingDecoder::pingPlayback(std::shared_ptr<liboculus::SimplePingResult> ping) {

  const int nBearings = ping->ping()->nBeams;
  const int nRanges = ping->ping()->nRanges;

  float bearings[nBearings];
  float ranges[nRanges];
  float intensities[nRanges * nBearings];
  //
  for (unsigned int i = 0; i < nBearings; i++) {
    bearings[i] = ping->bearings().at(i);
  }
  for (unsigned int i = 0; i < nRanges; i++) {
    ranges[i] = float(i + 0.5) * ping->ping()->rangeResolution;
  }
  for (unsigned int i = 0; i < nBearings; i++) {
    for (unsigned int j = 0; j < nRanges; j++) {
      intensities[i] = ping->image().at(i, j);
    }
  }
  //
  float frequency = ping->ping()->frequency;
  ;
  //
  sonarData.frequency = frequency;
  sonarData.bearings = bearings;
  sonarData.ranges = ranges;
  sonarData.intensities = intensities;

  return sonarData;
}

} // namespace serdp_common
