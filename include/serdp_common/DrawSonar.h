#pragma once

#include <memory>

#include <opencv2/core/core.hpp>

#include "liboculus/SimplePingResult.h"
#include "serdp_common/DataStructures.h"
#include "serdp_common/ColorMaps.h"

namespace serdp_common {

  cv::Size calculateImageSize( const AbstractSonarInterface &ping, cv::Size hint, int pixPerRangeBin );

void drawSonar(const std::shared_ptr<liboculus::SimplePingResult> &ping, cv::Mat &mat, const SonarColorMap &colorMap=InfernoColorMap());
void drawSonar(const liboculus::SimplePingResult &ping, cv::Mat &mat, const SonarColorMap &colorMap = InfernoColorMap());

void drawSonar(const AbstractSonarInterface &ping, cv::Mat &mat, const SonarColorMap &colorMap=InfernoColorMap() );

} // namespace serdp_common
