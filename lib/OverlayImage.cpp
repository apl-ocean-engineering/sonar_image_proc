#include <opencv2/core.hpp>

namespace sonar_image_proc {

// Code from:
// https://jepsonsblog.blogspot.com/2012/10/overlay-transparent-image-in-opencv.html
//
// I've simplified the code by removing the "location" option.  The output will
// be the size of "background".  If foreground is smaller than background, it
// will only be mapped onto the upper-left corner of the output.
//
// \todo(@amarburg):  Optimize?  Loop unrolling?
//
void overlayImage(const cv::Mat &background, const cv::Mat &foreground,
                  cv::Mat &output) {
  background.copyTo(output);

  // start at the row indicated by location, or at row 0 if location.y is
  // negative.
  for (int fY = 0; fY < background.rows; ++fY) {

    // we are done of we have processed all rows of the foreground image.
    if (fY >= foreground.rows)
      break;

    // start at the column indicated by location,

    // or at column 0 if location.x is negative.
    for (int fX = 0; fX < background.cols; ++fX) {

      // we are done with this row if the column is outside of the foreground
      // image.
      if (fX >= foreground.cols)
        break;

      // determine the opacity of the foregrond pixel, using its fourth (alpha)
      // channel.
      double opacity =
          ((double)foreground
               .data[fY * foreground.step + fX * foreground.channels() + 3])

          / 255.;

      // and now combine the background and foreground pixel, using the opacity,

      // but only if opacity > 0.
      for (int c = 0; opacity > 0 && c < output.channels(); ++c) {
        unsigned char foregroundPx =
            foreground
                .data[fY * foreground.step + fX * foreground.channels() + c];
        unsigned char backgroundPx =
            background
                .data[fY * background.step + fX * background.channels() + c];
        output.data[fY * output.step + output.channels() * fX + c] =
            backgroundPx * (1. - opacity) + foregroundPx * opacity;
      }
    }
  }
}
}; // namespace sonar_image_proc