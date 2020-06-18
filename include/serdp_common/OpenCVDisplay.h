#pragma once

#include <functional>
#include <memory>

#include <opencv2/core/core.hpp>

#include "OpenCVDisplay.h"
#include "active_object/active.h"
#include "liboculus/SimplePingResult.h"

namespace serdp_common {

using std::vector;

class OpenCVDisplay {
public:
  OpenCVDisplay(std::function<void(const char)> callback);

  bool setEnabled(bool e) { return _enabled = e; }

  void showVideo(const cv::Mat &mat, const std::string &name = "Image") {
    if (_enabled) {
        vector<cv::Mat> vec;
        vec.push_back(mat);
      _thread->send(std::bind(&OpenCVDisplay::implShowVideo, this, vec, name));
    }
  }

  void showVideo(vector<cv::Mat> mats, const std::string &name = "Image") {
    if (_enabled)
      _thread->send(std::bind(&OpenCVDisplay::implShowVideo, this, mats, name));
  }

  void showSonar(const liboculus::SimplePingResult &ping, const std::string &name = "Sonar") {
    if (_enabled)
      _thread->send(std::bind(&OpenCVDisplay::implShowSonar, this, ping, name));
  }

  float setPreviewScale(float scale) {
    _previewScale = scale;
    return _previewScale;
  }

  //cv::Mat sonarPing2Img(const std::shared_ptr<liboculus::SimplePingResult> &ping);

protected:
  void implShowVideo(vector<cv::Mat> mats, const std::string &name = "Image");
  void implShowSonar(const liboculus::SimplePingResult ping, const std::string &name = "Image");

  void resizeImage(const cv::Mat &rawImage, cv::Mat &scaledImage);

private:
  bool _enabled;
  float _previewScale;

  std::unique_ptr<active_object::Active> _thread;
  std::function<void(const char)> _keyHandler;
};

} // namespace serdp_common
