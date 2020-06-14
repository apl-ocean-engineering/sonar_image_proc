
#include "serdp_common/DrawSonar.h"

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

#ifndef THETA_SHIFT
#define THETA_SHIFT PI;
#endif

using namespace std;
using namespace liboculus;
using namespace cv;

namespace serdp_common {

void drawSonar(const shared_ptr<SimplePingResult> &ping, Mat &mat) {
  drawSonar(*ping, mat);
}

void drawSonar(const SimplePingResult &ping, Mat &mat) {

  LOG(DEBUG) << "Theta shift " << THETA_SHIFT;

  // Ensure mat is 8UC3
  mat.create(mat.size(), CV_8UC3);
  // mat.setTo( cv::Vec3b(0,0,0) );

  const unsigned int radius = mat.size().width / 2;
  const cv::Point origin(radius, mat.size().height);

  const float binThickness = 3 * ceil(radius / ping.oculusPing()->nRanges);

  LOG(DEBUG) << "binThickness is " << binThickness;

  // Build vector of start and end angles
  // (in degrees, with sonar 0 == straight ahead)

  vector<pair<float, float>> angles(ping.oculusPing()->nBeams, make_pair(0.0f, 0.0f));
  for (unsigned int b = 0; b < ping.oculusPing()->nBeams; ++b) {
    float begin = 0.0, end = 0.0;

    // LOG(DEBUG) << "Bearing " << b << " is " << ping.bearings().at(b);
    if (b == 0) {

      end = (ping.bearings().at(b + 1) + ping.bearings().at(b)) / 2.0;
      begin = 2 * ping.bearings().at(b) - end;

    } else if (b == ping.oculusPing()->nBeams - 1) {

      begin = angles[b - 1].second;
      end = 2 * ping.bearings().at(b) - begin;

    } else {

      begin = angles[b - 1].second;
      end = (ping.bearings().at(b + 1) + ping.bearings().at(b)) / 2.0;
    }

    // LOG(DEBUG) << "Bin " << b << " from " << begin << " to " << end;

    angles[b] = make_pair(begin, end);
  }

  // Bearings
  std::vector<float> bearings;
  std::vector<float> ranges;
  for (unsigned int i = 0; i < ping.oculusPing()->nBeams; i++) {
    bearings.push_back(ping.bearings().at(i) + THETA_SHIFT);
  }
  // Ranges
  for (unsigned int i = 0; i < ping.oculusPing()->nRanges; i++) {
    ranges.push_back(float(i + 0.5) * ping.oculusPing()->rangeResolution);
  }

  const auto [bearingMin, bearingMax] =
      std::minmax_element(begin(bearings), end(bearings));
  const auto [rangeMin, rangeMax] =
      std::minmax_element(begin(ranges), end(ranges));

  for (unsigned int r = 0; r < ping.oculusPing()->nRanges; ++r) {
    for (unsigned int b = 0; b < ping.oculusPing()->nBeams; ++b) {

      float bearing = bearings.at(b);
      float range = ranges.at(r);
      float intensity = ping.image().at(b, r);
      // Insert color mapping here
      // cv::Scalar color(intensity, intensity, intensity);
      cv::Scalar color(bearing * SCALE_FACTOR, range * SCALE_FACTOR,
                       intensity * SCALE_FACTOR);

      const float begin = angles[b].first + 270, end = angles[b].second + 270;

      // LOG_IF( DEBUG, r == 128 ) << "From " << begin << " to " << end << ";
      // color " << color;

      const float rad = float(radius * r) / ping.oculusPing()->nRanges;

      const float fudge = 0.7;

      // Assume angles are in image frame x-right, y-down
      cv::ellipse(mat, origin, cv::Size(rad, rad), 0, begin - fudge,
                  end + fudge, color, binThickness * 1.4);
    }
  }
}

} // namespace serdp_common
