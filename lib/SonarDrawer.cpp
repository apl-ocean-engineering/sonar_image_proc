// Copyright 2021 University of Washington Applied Physics Laboratory
//

#include "ros/console.h"

#include "sonar_image_proc/DrawSonar.h"

#include <iostream>
#include <limits>
#include <opencv2/imgproc/imgproc.hpp>

namespace sonar_image_proc {

using sonar_image_proc::AbstractSonarInterface;

SonarDrawer::SonarDrawer() { ; }

cv::Mat SonarDrawer::drawRectSonarImage(const AbstractSonarInterface &ping,
                                        const SonarColorMap &colorMap,
                                        const cv::Mat &rectIn) {
  cv::Mat rect(rectIn);

  const cv::Size imgSize(ping.nRanges(), ping.nBearings());

  if ((rect.type() == CV_8UC3) || (rect.type() == CV_32FC3) ||
      (rect.type() == CV_32FC1)) {
    rect.create(imgSize, rect.type());
  } else {
    rect.create(imgSize, CV_8UC3);
  }

  for (int r = 0; r < ping.nRanges(); r++) {
    for (int b = 0; b < ping.nBearings(); b++) {
      const AzimuthRangeIndices loc(b, r);

      if (rect.type() == CV_8UC3) {
        rect.at<cv::Vec3b>(cv::Point(r, b)) = colorMap.lookup_cv8uc3(ping, loc);
      } else if (rect.type() == CV_32FC3) {
        rect.at<cv::Vec3f>(cv::Point(r, b)) =
            colorMap.lookup_cv32fc3(ping, loc);
      } else if (rect.type() == CV_32FC1) {
        rect.at<float>(cv::Point(r, b)) = colorMap.lookup_cv32fc1(ping, loc);
      } else {
        assert("Should never get here.");
      }
    }
  }

  return rect;
}

cv::Mat SonarDrawer::remapRectSonarImage(const AbstractSonarInterface &ping,
                                         const cv::Mat &rectImage) {
  cv::Mat out;
  const CachedMap::MapPair maps(_map(ping));
  cv::remap(rectImage, out, maps.first, maps.second, cv::INTER_CUBIC,
            cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

  return out;
}

cv::Mat SonarDrawer::drawOverlay(const AbstractSonarInterface &ping,
                                 const cv::Mat &sonarImage) {
  cv::Mat out(sonarImage);

  const cv::Vec3b lineColor(255, 255, 255);
  const int lineThickness = 2;

  const float minAzimuth = ping.minAzimuth();
  const float maxAzimuth = ping.maxAzimuth();

  const float maxRange = ping.maxRange();

  const cv::Point2f origin(sonarImage.size().width / 2,
                           sonarImage.size().height);

  //== Draw arcs ==
  const float arcSpacing = 1; // meters ... this should be automatic
  const float minRange = 1;   // meters

  const float minRangePix = (minRange / maxRange) * sonarImage.size().height;

  for (float r = minRange; r < ping.maxRange(); r += arcSpacing) {
    const float radiusPix = (r / maxRange) * sonarImage.size().height;

    ROS_INFO_STREAM("Range " << r << " at " << radiusPix << "pix");

    cv::ellipse(out, origin, cv::Size(radiusPix, radiusPix), 0,
                180 / M_PI * (-M_PI / 2 + minAzimuth),
                180 / M_PI * (-M_PI / 2 + maxAzimuth), lineColor,
                lineThickness);
  }

  cv::ellipse(out, origin,
              cv::Size(sonarImage.size().height, sonarImage.size().height), 0,
              0, 360, lineColor, lineThickness);

  //== Draw radials ==
  std::vector<float> radials;

  // Configuration ... move later
  const float radialsSpacing = 10 * M_PI / 180.0; // degrees
  const bool centered = false;

  radials.push_back(minAzimuth);
  radials.push_back(maxAzimuth);

  // I kindof assume minAzimuth is < 0

  if (centered) {
    //
  } else {
    for (float d = radialsSpacing / 2; d > minAzimuth && d < maxAzimuth;
         d += radialsSpacing) {
      radials.push_back(d);
    }
    for (float d = -radialsSpacing / 2; d > minAzimuth && d < maxAzimuth;
         d -= radialsSpacing) {
      radials.push_back(d);
    }
  }

  // Sort
  std::sort(radials.begin(), radials.end());
  auto last = std::unique(radials.begin(), radials.end());
  radials.erase(last, radials.end());

  // And draw
  ROS_INFO_STREAM("Drawing " << radials.size() << " radials");
  for (const auto b : radials) {

    const float angle = -M_PI / 2 + b;

    ROS_INFO_STREAM("  : " << b << "  -->  " << angle);

    const cv::Point2f begin(minRangePix * cos(angle) + origin.x,
                            minRangePix * sin(angle) + origin.y);
    const cv::Point2f end(sonarImage.size().height * cos(angle) + origin.x,
                          sonarImage.size().height * sin(angle) + origin.y);

    ROS_INFO_STREAM("     : from " << begin.x << ", " << begin.y << "  to  "
                                   << end.x << ", " << end.y);
    cv::line(out, origin, end, lineColor, lineThickness);
  }

  return out;
}

cv::Mat SonarDrawer::drawSonar(const AbstractSonarInterface &ping,
                               const SonarColorMap &colorMap,
                               const cv::Mat &img, bool addOverlay) {
  cv::Mat rect = drawRectSonarImage(ping, colorMap, img);
  cv::Mat sonar = remapRectSonarImage(ping, rect);
  if (addOverlay) {
    return drawOverlay(ping, sonar);
  } else {
    return sonar;
  }
}

// ==== SonarDrawer::CachedMap ====

SonarDrawer::CachedMap::MapPair SonarDrawer::CachedMap::
operator()(const AbstractSonarInterface &ping) {
  // _scMap[12] are mutable to break out of const
  if (!isValid(ping))
    create(ping);

  return std::make_pair(_scMap1, _scMap2);
}

//  **assumes** this structure for the rectImage:
//   * It has nBearings cols and nRanges rows
//
void SonarDrawer::CachedMap::create(const AbstractSonarInterface &ping) {
  cv::Mat newmap;

  const int nRanges = ping.nRanges();
  const auto azimuthBounds = ping.azimuthBounds();

  const int minusWidth = floor(nRanges * sin(azimuthBounds.first));
  const int plusWidth = ceil(nRanges * sin(azimuthBounds.second));
  const int width = plusWidth - minusWidth;

  const int originx = abs(minusWidth);

  const cv::Size imgSize(width, nRanges);
  if ((width <= 0) || (nRanges <= 0))
    return;

  newmap.create(imgSize, CV_32FC2);

  const float db =
      (azimuthBounds.second - azimuthBounds.first) / ping.nAzimuth();

  for (int x = 0; x < newmap.cols; x++) {
    for (int y = 0; y < newmap.rows; y++) {
      // For cv::remap, a map is
      //
      //  dst = src( mapx(x,y), mapy(x,y) )
      //
      // That is, the map is the size of the dst array,
      // and contains the coords in the source image
      // for each pixel in the dst image.
      //
      // This map draws the sonar with range = 0
      // centered on the bottom edge of the resulting image
      // with increasing range along azimuth = 0 going
      // vertically upwards in the image

      // Calculate range and bearing of this pixel from origin
      const float dx = x - originx;
      const float dy = newmap.rows - y;

      const float range = sqrt(dx * dx + dy * dy);
      const float azimuth = atan2(dx, dy);

      float xp = range;

      //\todo This linear algorithm is not robust if the azimuths
      // are non-linear.   Should implement a real interpolation...
      float yp = (azimuth - azimuthBounds.first) / db;

      newmap.at<cv::Vec2f>(cv::Point(x, y)) = cv::Vec2f(xp, yp);
    }
  }

  cv::convertMaps(newmap, cv::Mat(), _scMap1, _scMap2, CV_16SC2);

  // Save meta-information to check for cache expiry
  _numRanges = ping.nRanges();
  _numAzimuth = ping.nBearings();

  _rangeBounds = ping.rangeBounds();
  _azimuthBounds = ping.azimuthBounds();
}

bool SonarDrawer::CachedMap::isValid(const AbstractSonarInterface &ping) const {
  if (_scMap1.empty() || _scMap2.empty())
    return false;

  // Check for cache invalidation...
  if ((_numAzimuth != ping.nAzimuth()) || (_numRanges != ping.nRanges()) ||
      (_rangeBounds != ping.rangeBounds() ||
       (_azimuthBounds != ping.azimuthBounds())))
    return false;

  return true;
}

} // namespace sonar_image_proc
