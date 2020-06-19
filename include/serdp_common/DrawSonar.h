#pragma once

#include <memory>

#include <opencv2/core/core.hpp>

#include "liboculus/SimplePingResult.h"
#include "serdp_common/DataStructures.h"
#include "serdp_common/ColorMaps.h"

namespace serdp_common {

cv::Size calculateImageSize( const AbstractSonarInterface &ping, cv::Size hint, int pixPerRangeBin = 2);

// Helper wrappers
inline cv::Size calculateImageSize( const liboculus::SimplePingResult &ping, cv::Size hint, int pixPerRangeBin = 2) {
  return calculateImageSize( SimplePingResultInterface(ping), hint, pixPerRangeBin );
}




void drawSonar(const AbstractSonarInterface &ping, cv::Mat &mat, const SonarColorMap &colorMap=InfernoColorMap() );

// Helper wrappers
inline void drawSonar(const std::shared_ptr<liboculus::SimplePingResult> &ping, cv::Mat &mat, const SonarColorMap &colorMap=InfernoColorMap()) {
  drawSonar( SimplePingResultInterface(*ping), mat, colorMap );
}

inline void drawSonar(const liboculus::SimplePingResult &ping, cv::Mat &mat, const SonarColorMap &colorMap = InfernoColorMap()) {
  drawSonar( SimplePingResultInterface(ping), mat, colorMap);
}



} // namespace serdp_common
