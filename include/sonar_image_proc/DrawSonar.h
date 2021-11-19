// Copyright 2021 University of Washington Applied Physics Laboratory
//

#pragma once

#include <memory>

#include <opencv2/core/core.hpp>

#include "sonar_image_proc/ColorMaps.h"
#include "sonar_image_proc/AbstractSonarInterface.h"

namespace sonar_image_proc {

void drawSonar(const sonar_image_proc::AbstractSonarInterface &ping,
                    cv::Mat &image,
                    const SonarColorMap &colorMap = InfernoColorMap(),
                    const cv::Mat &rectImage = cv::Mat());

// Maps the sonar ping to an RGB image.
// rectImage is reshaped to be numRanges rows x numBearings columns
//
// If rectImage is either 8UC3 or 32FC3, it retains that type, otherwise
// rectImage is converted to 8UC3
//
// Cell (0,0) is the color mapping of the data with the smallest range and
// smallest (typically, most negative) bearing in the ping.
//
// Cell (nRange,0) is the data at the max range, most negative bearing
//
// Cell (nRange,nBearing) is the data at the max range, most positive bearing
//
void drawSonarRectImage(const sonar_image_proc::AbstractSonarInterface &ping,
                    cv::Mat &rectImage,
                    const SonarColorMap &colorMap = InfernoColorMap());


namespace old_api {

// === Old / Legacy API below ===

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

}

}  // namespace sonar_image_proc
