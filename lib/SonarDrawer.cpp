// Copyright 2021 University of Washington Applied Physics Laboratory
//

#include "sonar_image_proc/DrawSonar.h"
#include "sonar_image_proc/OverlayImage.h"

#include <iostream>
#include <limits>
#include <opencv2/imgproc/imgproc.hpp>

namespace sonar_image_proc {

using sonar_image_proc::AbstractSonarInterface;

static float deg2radf(float deg) { return deg * M_PI / 180.0; }
static float rad2degf(float rad) { return rad * 180.0 / M_PI; }

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
  // Alpha blend overlay onto sonarImage
  cv::Mat output;
  overlayImage<unsigned char>(
      sonarImage, _overlay(ping, sonarImage, overlayConfig()), output);

  return output;
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

// ==== SonarDrawer::Cached ====

bool SonarDrawer::Cached::isValid(const AbstractSonarInterface &ping) const {
  // Check for cache invalidation...
  if ((_numAzimuth != ping.nAzimuth()) || (_numRanges != ping.nRanges()) ||
      (_rangeBounds != ping.rangeBounds() ||
       (_azimuthBounds != ping.azimuthBounds())))
    return false;

  return true;
}

// ==== SonarDrawer::CachedMap ====

SonarDrawer::CachedMap::MapPair
SonarDrawer::CachedMap::operator()(const AbstractSonarInterface &ping) {
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

  return Cached::isValid(ping);
}

// === SonarDrawer::CachedOverlay ===

bool SonarDrawer::CachedOverlay::isValid(const AbstractSonarInterface &ping,
                                         const cv::Mat &sonarImage,
                                         const OverlayConfig &config) const {
  if (sonarImage.size() != _overlay.size())
    return false;

  if (_config_used != config)
    return false;

  return Cached::isValid(ping);
}

const cv::Mat &
SonarDrawer::CachedOverlay::operator()(const AbstractSonarInterface &ping,
                                       const cv::Mat &sonarImage,
                                       const OverlayConfig &config) {
  if (!isValid(ping, sonarImage, config))
    create(ping, sonarImage, config);

  return _overlay;
}

// Converts sonar bearing (with sonar "forward" at bearing 0) to image
// orientation with sonar "forward" point upward in the image, which is the -Y
// direction in image coordinates.
static float bearingToImage(float d) { return (-M_PI / 2) + d; }

void SonarDrawer::CachedOverlay::create(const AbstractSonarInterface &ping,
                                        const cv::Mat &sonarImage,
                                        const OverlayConfig &config) {

  const cv::Size sz(sonarImage.size());
  const cv::Point2f origin(sz.width / 2, sz.height);

  // Reset overlay
  _overlay = cv::Mat::zeros(sz, CV_8UC4);
  const cv::Vec3b color(config.lineColor());
  const cv::Vec4b textColor(color[0], color[1], color[2], 255);
  const cv::Vec4b lineColor(color[0], color[1], color[2],
                            config.lineAlpha() * 255);

  const float minAzimuth = ping.minAzimuth();
  const float maxAzimuth = ping.maxAzimuth();

  const float maxRange = ping.maxRange();

  //== Draw arcs ==
  float arcSpacing = config.rangeSpacing();

  if (arcSpacing <= 0) {
    // Calculate automatically .. just a lame heuristic for now
    if (maxRange < 2)
      arcSpacing = 0.5;
    else if (maxRange < 5)
      arcSpacing = 1.0;
    else if (maxRange < 10)
      arcSpacing = 2.0;
    else if (maxRange < 50)
      arcSpacing = 10.0;
    else
      arcSpacing = 20.0;
  }

  const float minRange = arcSpacing;

  for (float r = minRange; r < ping.maxRange(); r += arcSpacing) {
    const float radiusPix = (r / maxRange) * sonarImage.size().height;

    cv::ellipse(_overlay, origin, cv::Size(radiusPix, radiusPix), 0,
                rad2degf(bearingToImage(minAzimuth)),
                rad2degf(bearingToImage(maxAzimuth)), lineColor,
                config.lineThickness());

    {
      std::stringstream rstr;
      rstr << r;

      // Calculate location of string
      const float theta = bearingToImage(minAzimuth);

      // \todo{??} Should calculate this automatically ... not sure
      const cv::Point2f offset(-25, 20);

      const cv::Point2f pt(radiusPix * cos(theta) + origin.x + offset.x,
                           radiusPix * sin(theta) + origin.y + offset.y);

      cv::putText(_overlay, rstr.str(), pt, cv::FONT_HERSHEY_PLAIN,
                  config.fontScale(), textColor);
    }
  }

  // And one arc at max range
  cv::ellipse(_overlay, origin, sz, 0, rad2degf(bearingToImage(minAzimuth)),
              rad2degf(bearingToImage(maxAzimuth)), lineColor,
              config.lineThickness());

  //== Draw radials ==
  std::vector<float> radials;

  // Configuration ... move later
  const float radialSpacing = deg2radf(config.radialSpacing());

  radials.push_back(minAzimuth);
  radials.push_back(maxAzimuth);

  // If radialSpacing == 0. draw only the outline radial lines
  if (radialSpacing > 0) {
    if (config.radialAtZero()) {
      // \todo(@amarburg) to implement
    } else {
      for (float d = radialSpacing / 2; d > minAzimuth && d < maxAzimuth;
           d += radialSpacing) {
        radials.push_back(d);
      }
      for (float d = -radialSpacing / 2; d > minAzimuth && d < maxAzimuth;
           d -= radialSpacing) {
        radials.push_back(d);
      }
    }
  }

  // Sort and unique
  std::sort(radials.begin(), radials.end());
  auto last = std::unique(radials.begin(), radials.end());
  radials.erase(last, radials.end());

  // And draw
  const float minRangePix = (minRange / maxRange) * sz.height;

  if (minRange < maxRange) {
    for (const auto b : radials) {
      const float theta = bearingToImage(b);

      const cv::Point2f begin(minRangePix * cos(theta) + origin.x,
                              minRangePix * sin(theta) + origin.y);
      const cv::Point2f end(sz.height * cos(theta) + origin.x,
                            sz.height * sin(theta) + origin.y);

      cv::line(_overlay, begin, end, lineColor, config.lineThickness());
    }
  }

  _config_used = config;
}

} // namespace sonar_image_proc
