#pragma once

#include <memory>

#include <opencv2/core/core.hpp>

#include "draw_sonar/DataStructures.h"
#include "draw_sonar/ColorMaps.h"

namespace draw_sonar {

cv::Size calculateImageSize( const AbstractSonarInterface &ping, cv::Size hint, int pixPerRangeBin = 2);

// Helper wrappers

void drawSonar(const AbstractSonarInterface &ping, cv::Mat &mat, const SonarColorMap &colorMap=InfernoColorMap() );

} // namespace serdp_common
