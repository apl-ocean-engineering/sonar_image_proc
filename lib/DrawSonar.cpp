// Copyright 2021 University of Washington Applied Physics Laboratory
//

#include "sonar_image_proc/DrawSonar.h"

#include <iostream>
#include <limits>
#include <opencv2/imgproc/imgproc.hpp>

namespace sonar_image_proc {

using namespace std;
using namespace cv;

void drawSonarRectImage(const sonar_image_proc::AbstractSonarInterface &ping,
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
            rect.at<Vec3b>(cv::Point(r, b)) = colorMap.lookup<Vec3b>(ping, b, r);
        } else if (rect.type() == CV_32FC3) {
            rect.at<Vec3f>(cv::Point(r, b)) = colorMap.lookup<Vec3f>(ping, b, r);
        } else {
            assert("Should never get here.");
        }
    }
    }
}

void drawSonar(const sonar_image_proc::AbstractSonarInterface &ping,
                    cv::Mat &img,
                    const SonarColorMap &colorMap) {

  cv::Mat rect;
  drawSonarRectImage(ping, rect, colorMap);

  const int nRanges = ping.nRanges();

  // Find min and max bearing
  float minBearing = std::numeric_limits<float>::max();
  float maxBearing = -std::numeric_limits<float>::max();

  for (int i = 0; i < ping.nBearings(); i++) {
    const float b = ping.bearing(i);

    if (b > maxBearing) maxBearing = b;
    if (b < minBearing) minBearing = b;
  }

  const int minusWidth = floor(nRanges * sin(minBearing));
  const int plusWidth = ceil(nRanges * sin(maxBearing));
  const int width = plusWidth - minusWidth;

  const int originx = abs(minusWidth);

  const cv::Size imgSize(width,nRanges);

  const float db = (ping.bearing(ping.nBearings()-1) - ping.bearing(0)) / ping.nBearings();

  // Create map
  cv::Mat mm(imgSize, CV_32FC2);
  for (int x=0; x<mm.cols; x++) {
    for (int y=0; y<mm.rows; y++) {
      // Unoptimized version to start

      // Map is
      //
      //  dst = src( mapx(x,y), mapy(x,y) )
      //
      float xp, yp;

      // Calculate range and bearing of this pixel from origin
      const float dx = x-originx;
      const float dy = mm.rows-y;

      const float range = sqrt( dx*dx + dy*dy );
      const float azimuth = atan2(dx,dy);

      // yp is simply range
      xp = range;
      yp = (azimuth - ping.bearing(0))/db;

      mm.at<Vec2f>( cv::Point(x,y) ) = Vec2f(xp,yp);
    }
  }

  cv::remap(rect, img, mm, cv::Mat(),
            cv::INTER_CUBIC, cv::BORDER_CONSTANT, 
            cv::Scalar(0, 0, 0));



}


}  // namespace sonar_image_proc
