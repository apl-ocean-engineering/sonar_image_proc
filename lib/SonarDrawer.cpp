// Copyright 2021 University of Washington Applied Physics Laboratory
//

#include "sonar_image_proc/DrawSonar.h"

#include <iostream>
#include <limits>
#include <opencv2/imgproc/imgproc.hpp>

namespace sonar_image_proc {

SonarDrawer::SonarDrawer()
{;}

void SonarDrawer::drawSonarRectImage(const sonar_image_proc::AbstractSonarInterface &ping,
                        cv::Mat &rect,
                        const SonarColorMap &colorMap) {
    if ((rect.type() == CV_8UC3) || (rect.type() == CV_32FC2)) {
        rect.create(cv::Size(ping.nRanges(), ping.nBearings()),
            rect.type());
    } else {
        rect.create(cv::Size(ping.nRanges(), ping.nBearings()),
                    CV_8UC3);
    }
    rect.setTo(cv::Vec3b(0, 0, 0));

    for (int r = 0; r < ping.nRanges(); r++) {
    for (int b = 0; b < ping.nBearings(); b++) {

        if (rect.type() == CV_8UC3) {
            rect.at<cv::Vec3b>(cv::Point(r, b)) = colorMap.lookup<cv::Vec3b>(ping, b, r);
        } else if (rect.type() == CV_32FC3) {
            rect.at<cv::Vec3f>(cv::Point(r, b)) = colorMap.lookup<cv::Vec3f>(ping, b, r);
        } else {
            assert("Should never get here.");
        }
    }
    }
}

void SonarDrawer::drawSonar(const sonar_image_proc::AbstractSonarInterface &ping,
                    cv::Mat &img,
                    const SonarColorMap &colorMap,
                    const cv::Mat &rect) {
  cv::Mat rectImage(rect);

  if (rect.empty())
      drawSonarRectImage(ping, rectImage, colorMap);

    const CachedMap::MapPair maps(_map(ping));
  cv::remap(rect, img, maps.first, maps.second,
            cv::INTER_CUBIC, cv::BORDER_CONSTANT,
            cv::Scalar(0, 0, 0));
}



// ==== SonarDrawer::CachedMap ====

SonarDrawer::CachedMap::MapPair SonarDrawer::CachedMap::operator()(const sonar_image_proc::AbstractSonarInterface &ping) {
    // Break const to update the cache
    if (!isValid(ping)) create(ping);

    return std::make_pair(_scMap1, _scMap2);
}


// Create **assumes** the structure of the rectImage:
//   ** It has nBearings cols and nRanges rows
//
void SonarDrawer::CachedMap::create(const sonar_image_proc::AbstractSonarInterface &ping) {
  cv::Mat newmap;

  const int nRanges = ping.nRanges();
  const auto azimuthBounds = ping.azimuthBounds();

  const int minusWidth = floor(nRanges * sin(azimuthBounds.first));
  const int plusWidth = ceil(nRanges * sin(azimuthBounds.second));
  const int width = plusWidth - minusWidth;

  const int originx = abs(minusWidth);

  const cv::Size imgSize(width, nRanges);
  newmap.create(imgSize, CV_32FC2);

  const float db = (azimuthBounds.second - azimuthBounds.first) / ping.nAzimuth();

  for (int x=0; x < newmap.cols; x++) {
    for (int y=0; y < newmap.rows; y++) {
      // Unoptimized version to start

      // Map is
      //
      //  dst = src( mapx(x,y), mapy(x,y) )
      //
      float xp, yp;

      // Calculate range and bearing of this pixel from origin
      const float dx = x-originx;
      const float dy = newmap.rows-y;

      const float range = sqrt(dx*dx + dy*dy);
      const float azimuth = atan2(dx, dy);

      // yp is simply range
      xp = range;
      yp = (azimuth - ping.bearing(0))/db;

      newmap.at<cv::Vec2f>(cv::Point(x, y)) = cv::Vec2f(xp, yp);
    }
  }

  // Save metadata
  cv::convertMaps(newmap, cv::Mat(), _scMap1, _scMap2, CV_16SC2);

  _numRanges = ping.nRanges();
  _numAzimuth = ping.nBearings();

  _rangeBounds = ping.rangeBounds();
  _azimuthBounds = ping.azimuthBounds();
}

bool SonarDrawer::CachedMap::isValid(const sonar_image_proc::AbstractSonarInterface &ping) const {
    if (_scMap1.empty() || _scMap2.empty()) return false;

    // Check for cache invalidation...
    if ((_numAzimuth != ping.nAzimuth()) ||
        (_numRanges != ping.nRanges()) ||
        (_rangeBounds != ping.rangeBounds() ||
        (_azimuthBounds != ping.azimuthBounds()))) return false;

    return true;
}



}  // namespace sonar_image_proc
