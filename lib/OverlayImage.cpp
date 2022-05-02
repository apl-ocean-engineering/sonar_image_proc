#include <opencv2/core.hpp>

namespace sonar_image_proc {

// Code from:
// https://jepsonsblog.blogspot.com/2012/10/overlay-transparent-image-in-opencv.html
//
// \todo(@amarburg):  Optimize?  Loop unrolling?
//
void overlayImage(const cv::Mat &background, const cv::Mat &foreground,
                  cv::Mat &output) {
  const cv::Point2i location(0, 0);

  background.copyTo(output);

  // start at the row indicated by location, or at row 0 if location.y is
  // negative.
  for (int y = std::max(location.y, 0); y < background.rows; ++y) {
    int fY = y - location.y; // because of the translation

    // we are done of we have processed all rows of the foreground image.
    if (fY >= foreground.rows)
      break;

    // start at the column indicated by location,

    // or at column 0 if location.x is negative.
    for (int x = std::max(location.x, 0); x < background.cols; ++x) {
      int fX = x - location.x; // because of the translation.

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
                .data[y * background.step + x * background.channels() + c];
        output.data[y * output.step + output.channels() * x + c] =
            backgroundPx * (1. - opacity) + foregroundPx * opacity;
      }
    }
  }
}
}; // namespace sonar_image_proc