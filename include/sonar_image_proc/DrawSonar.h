#pragma once

#include <memory>

#include <opencv2/core/core.hpp>

#include "sonar_image_proc/ColorMaps.h"

#include "sonar_image_proc/AbstractSonarInterface.h"

namespace sonar_image_proc {

cv::Size calculateImageSize( const sonar_image_proc::AbstractSonarInterface &ping, cv::Size hint, int pixPerRangeBin = 2, float maxRange = -1.0 );

void drawSonar(const sonar_image_proc::AbstractSonarInterface &ping, cv::Mat &mat, const SonarColorMap &colorMap=InfernoColorMap(), float maxRange = -1.0 );

}
