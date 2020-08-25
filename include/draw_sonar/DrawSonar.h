#pragma once

#include <memory>

#include <opencv2/core/core.hpp>

#include "draw_sonar/DataStructures.h"
#include "draw_sonar/ColorMaps.h"

#include "imaging_sonar_msgs/AbstractSonarInterface.h"

namespace draw_sonar {

cv::Size calculateImageSize( const imaging_sonar_msgs::AbstractSonarInterface &ping, cv::Size hint, int pixPerRangeBin = 2, float maxRange = -1.0 );

void drawSonar(const imaging_sonar_msgs::AbstractSonarInterface &ping, cv::Mat &mat, const SonarColorMap &colorMap=InfernoColorMap(), float maxRange = -1.0 );

}
