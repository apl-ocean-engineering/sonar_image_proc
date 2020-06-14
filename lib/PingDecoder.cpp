#include "serdp_common/PingDecoder.h"

namespace serdp_common {

std::shared_ptr<SonarData>
PingDecoder::pingPlayback(std::shared_ptr<liboculus::SimplePingResult> ping) {

  const unsigned int nBearings = ping->oculusPing()->nBeams;
  const unsigned int nRanges = ping->oculusPing()->nRanges;

  std::vector<float> bearings;
  std::vector<float> ranges;
  std::vector<float> intensities;
  //
  for (unsigned int i = 0; i < nBearings; i++) {
    bearings.push_back(ping->bearings().at(i));
    // std::cout << "bearing: " << ping->bearings().at(i) << std::endl;
  }
  for (unsigned int i = 0; i < nRanges; i++) {
    ranges.push_back(float(i + 0.5) * ping->oculusPing()->rangeResolution);
  }
  for (unsigned int i = 0; i < nRanges; i++) {
    for (unsigned int j = 0; j < nBearings; j++) {
      intensities.push_back(ping->image().at(j, i));
    }
  }
  std::shared_ptr<SonarData> sonarData(new SonarData);
  //
  float frequency = ping->oculusPing()->frequency;
  float timestamp = ping->oculusPing()->pingStartTime;

  sonarData->nBearings = nBearings;

  sonarData->nRanges = nRanges;
  sonarData->timestamp = timestamp;
  sonarData->frequency = frequency;
  sonarData->bearings = bearings;
  sonarData->ranges = ranges;
  sonarData->intensities = intensities;

  return sonarData;
}

} // namespace serdp_common
