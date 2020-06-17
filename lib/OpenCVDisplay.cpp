
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "serdp_common/OpenCVDisplay.h"

#include "serdp_common/DrawSonar.h"

namespace serdp_common {

using namespace liboculus;

OpenCVDisplay::OpenCVDisplay(std::function<void(const char)> keyHandler)
    : _enabled(false), _previewScale(0.25),
      _thread(active_object::Active::createActive()),
      _keyHandler(keyHandler)
{}

//=== Functions related to showing video =====

void OpenCVDisplay::implShowVideo(vector<cv::Mat> rawImages) {
  const unsigned int numImages = rawImages.size();
  vector<cv::Mat> scaledImages(numImages);

  // Display images
  if (numImages == 1) {

    resizeImage(rawImages[0], scaledImages[0]);
    cv::imshow("Image", scaledImages[0]);

  } else if (numImages == 2) {

    std::thread resizeThread(&OpenCVDisplay::resizeImage, this,
                             std::cref(rawImages[1]),
                             std::ref(scaledImages[1]));
    resizeImage(rawImages[0], scaledImages[0]);
    resizeThread.join();

    cv::Mat composite(
        cv::Size(scaledImages[0].size().width + scaledImages[0].size().width,
                 std::max(scaledImages[0].size().height,
                          scaledImages[1].size().height)),
        scaledImages[0].type());

    if (!scaledImages[0].empty()) {
      cv::Mat leftROI(composite, cv::Rect(0, 0, scaledImages[0].size().width,
                                          scaledImages[0].size().height));
      scaledImages[0].copyTo(leftROI);
    }

    if (!scaledImages[1].empty()) {
      cv::Mat rightROI(composite, cv::Rect(scaledImages[0].size().width, 0,
                                           scaledImages[1].size().width,
                                           scaledImages[1].size().height));
      scaledImages[1].copyTo(rightROI);
    }

    cv::imshow("Composite", composite);
  }

  int c = cv::waitKey(1);
  if (_keyHandler)
    _keyHandler(c);
}

void OpenCVDisplay::resizeImage(const cv::Mat &rawImage, cv::Mat &scaledImage) {
  cv::resize(rawImage, scaledImage, cv::Size(), _previewScale, _previewScale);

  //				scaledImages[i] = cv::Mat(tmp.rows, tmp.cols,
  // CV_8UC3
  //);

  // Image from camera is BGRA  map to RGB
  // int from_to[] = { 0,0, 1,1, 2,2 };
  // cv::mixChannels( &tmp, 1, &(scaledImages[i]), 1, from_to, 3 );
  // cv::cvtColor( tmp, scaledImages[i], cv::COLOR_BGRA2RGB );
  // cv::extractChannel(tmp, scaledImages[i], 0 );
}

cv::Mat OpenCVDisplay::sonarPing2Img(
    const std::shared_ptr<liboculus::SimplePingResult> &ping) {
  cv::Mat mat(500, 1000, CV_8UC3);
  mat.setTo(cv::Vec3b(128, 128, 128));

  drawSonar(ping, mat );

  return mat;
}

//==== Functions related to showing sonar =====

void OpenCVDisplay::implShowSonar(
    const std::shared_ptr<SimplePingResult> &ping) {
  cv::Mat mat(500, 1000, CV_8UC3);
  mat.setTo(cv::Vec3b(128, 128, 128));

  drawSonar(ping, mat);
  cv::imshow("Sonar ping", mat);
}

} // namespace serdp_common
