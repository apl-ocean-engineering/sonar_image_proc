#pragma once

#include <memory>

#include <opencv2/core.hpp>

#include "liboculus/SimplePingResult.h"

#include "imaging_sonar_msgs/ImagingSonarMsg.h"

namespace serdp_common {

  void drawSonar( const std::shared_ptr<liboculus::SimplePingResult> &ping, cv::Mat &mat );

  void drawSonar(const imaging_sonar_msgs::ImagingSonarMsg& msg);
}
