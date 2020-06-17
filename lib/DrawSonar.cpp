
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

const float ThetaShift = 270;

void drawSonar(const shared_ptr<SimplePingResult> &ping, Mat &mat, const SonarColorMap &colorMap) {
  drawSonar( SimplePingResultInterface(*ping), mat, colorMap);
}

void drawSonar(const SimplePingResult &ping, Mat &mat, const SonarColorMap &colorMap) {
  drawSonar( SimplePingResultInterface(ping), mat, colorMap );
}

cv::Size calculateImageSize( const AbstractSonarInterface &ping, cv::Size hint, int pixPerRangeBin ) {

  int h = hint.height, w = hint.width;

  if( w <= 0 ) {

    if( h <= 0 ) {
        h = ping.nRanges() * pixPerRangeBin;
    }

    // Assume bearings are symmetric plus and minus
    // Also assumes bearings are degrees
    w = 2*ceil(fabs(h*sin( M_PI/180 * ping.bearing(0) )));

  } else if( h <= 0 ) {
    h = (w/2) / ceil(fabs(sin( M_PI/180 * ping.bearing(0) )));
  }

  return Size(w,h);
}

void drawSonar( const AbstractSonarInterface &ping, Mat &mat, const SonarColorMap &colorMap ) {

  // Ensure mat is 8UC3;
  mat.create(mat.size(), CV_8UC3);
  mat.setTo( cv::Vec3b(0,0,0) );

  const int nRanges = ping.nRanges();
  const int nBeams = ping.nBearings();

  const unsigned int radius = mat.size().height;
  const cv::Point origin(mat.size().width/2, mat.size().height);

  const float binThickness = 3 * ceil(radius / nRanges);

  // Current ImagingSonarMsg data is in _degrees_
  struct BearingEntry {
      float begin, center, end;

      BearingEntry( float b, float c, float e )
        : begin( b ), center(c), end(e)
          {;}
  };

  vector<BearingEntry> angles;
  angles.reserve( nBeams );

  for (unsigned int b = 0; b < nBeams; ++b) {
    const float center = ping.bearing(b);
    float begin = 0.0, end = 0.0;

    if (b == 0) {

      end = (ping.bearing(b + 1) + center) / 2.0;
      begin = 2 * center - end;

    } else if (b == nBeams - 1) {

      begin = angles[b - 1].end;
      end = 2 * center - begin;

    } else {

      begin = angles[b - 1].end;
      end = (ping.bearing(b + 1) + center) / 2.0;
    }

    angles.push_back( BearingEntry(begin, center, end) );
  }

  for (unsigned int r = 0; r < nRanges; ++r) {
    for (unsigned int b = 0; b < nBeams; ++b) {

      const float range = ping.range(r);
      const uint8_t intensity = ping.intensity(b,r);

      const float begin = angles[b].begin + ThetaShift,
                  end = angles[b].end + ThetaShift;

      const float rad = float(radius * r) / nRanges;

      const float fudge = 0; // in degrees

      // Assume angles are in image frame x-right, y-down
      cv::ellipse(mat, origin, cv::Size(rad, rad), 0,
                  begin, end,
                  255*colorMap( angles[b].center, range, intensity ),
                  binThickness * 1.4);
    }
  }
}

} // namespace serdp_common
