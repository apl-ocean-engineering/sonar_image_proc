// Copyright 2021 University of Washington Applied Physics Laboratory
//

#pragma once

#include <memory>

#include <opencv2/core/core.hpp>

#include "sonar_image_proc/ColorMaps.h"

#include "sonar_image_proc/AbstractSonarInterface.h"

namespace sonar_image_proc {

// Given an sonar image, calculates the bounding rectangle required to
// draw it.   Assumes zero range (the point of the fan) occurs on the
// bottom edge of the image.
// Azimuth = 0 is straight ahead from the sonar (the vertical axis in the image)
// Assuming azimuth = 0 is within the interval of [min azimuth, max azimuth]
// then the Image height is pixPerRangeBin * (max range in bin / range resolution)
// Image width is determined by the triangles defined by image height and
// min/max azimuth
//
// If set, maxRange is used in lieu of the ping's native max range,
// allowing truncation of the image
cv::Size calculateImageSize(const sonar_image_proc::AbstractSonarInterface &ping,
                            cv::Size hint,
                            int pixPerRangeBin = 2,
                            float maxRange = -1.0);

void drawSonar(const sonar_image_proc::AbstractSonarInterface &ping,
                            cv::Mat &mat,
                            const SonarColorMap &colorMap = InfernoColorMap(),
                            float maxRange = -1.0);

void drawSonarRect(const sonar_image_proc::AbstractSonarInterface &ping,
                            cv::Mat &rectImage,
                            const SonarColorMap &colorMap = InfernoColorMap(),
                            float maxRange = -1.0);


}  // namespace sonar_image_proc
