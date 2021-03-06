
#include "draw_sonar/DrawSonar.h"

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

#ifndef THETA_SHIFT
#define THETA_SHIFT PI;
#endif

using namespace std;
using namespace cv;

namespace draw_sonar {

using namespace imaging_sonar_msgs;
const float ThetaShift = 1.5*M_PI;

cv::Size calculateImageSize( const AbstractSonarInterface &ping, cv::Size hint, int pixPerRangeBin, float maxRange ) {

  int h = hint.height, w = hint.width;

  if( w <= 0 ) {

    if( h <= 0 ) {
      const float rangeMax = ( (maxRange > 0.0) ? maxRange : ping.maxRange() );
      const float rangeRes = (ping.maxRange() - ping.minRange()) / ping.nRanges();

      const int nEffectiveRanges = ceil( rangeMax / rangeRes);

      h = nEffectiveRanges * pixPerRangeBin;
    }

    // Assume bearings are symmetric plus and minus
    // Bearings must be radians
    w = 2*ceil(fabs(h*sin( ping.bearing(0) )));

  } else if( h <= 0 ) {
    h = (w/2) / ceil(fabs(sin( ping.bearing(0) )));
  }

  // Ensure w and h are both divisible by zero
  if( w % 2 ) w++;
  if( h % 2 ) h++;

  return Size(w,h);
}

void drawSonar( const AbstractSonarInterface &ping, Mat &mat, const SonarColorMap &colorMap, float maxRange ) {

  // Ensure mat is 8UC3;
  mat.create(mat.size(), CV_8UC3);
  mat.setTo( cv::Vec3b(0,0,0) );

  const int nRanges = ping.nRanges();
  const int nBeams = ping.nBearings();

  const float rangeMax = ( maxRange > 0.0 ? maxRange : ping.maxRange() );

  // Calculate effective range resolution of the output image.
  // The sensor's original resolution
  const float rangeRes = ( ping.maxRange() - ping.minRange() ) / ping.nRanges();
  // How many ranges are required to go from 0 to rangeMax (since the
  // sensor starts at some minimum)
  const int nEffectiveRanges = ceil(rangeMax / rangeRes);

  // Todo.  Calculate offset for non-zero minimum ranges
  const unsigned int radius = mat.size().height;
  // This effectively flips the origin, presumably so that it will appear
  // at the bottom fo the image.
  const cv::Point origin(mat.size().width/2, mat.size().height);

  // QUESTION: Why the factor of 2?
  // If I understand correctly, binThickness is the width of the range-bin, in pixels.
  const float binThickness = 2 * ceil(radius / nEffectiveRanges);

  struct BearingEntry {
      float begin, center, end;

      BearingEntry( float b, float c, float e )
        : begin( b ), center(c), end(e)
          {;}
  };

  vector<BearingEntry> angles;
  angles.reserve( nBeams );

  for ( int b = 0; b < nBeams; ++b ) {
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

  for ( int r = 0; r < nRanges; ++r ) {

    if( ping.range(r) > rangeMax ) continue;

    for ( int b = 0; b < nBeams; ++b ) {

      const float range = ping.range(r);
      const uint8_t intensity = ping.intensity(b,r);

      // QUESTION: Why are we rotating here?
      const float begin = angles[b].begin + ThetaShift,
                  end = angles[b].end + ThetaShift;

      const float rad = float(radius) * range/rangeMax;

      // Assume angles are in image frame x-right, y-down
      cv::ellipse(mat, origin, cv::Size(rad, rad), 0,
                  begin * 180/M_PI, end * 180/M_PI,
                  255*colorMap( angles[b].center, range, intensity ),
                  binThickness);
    }
  }
}

} // namespace serdp_common
