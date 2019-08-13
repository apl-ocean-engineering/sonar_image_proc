#pragma once

#include <memory>

#include <opencv2/core/core.hpp>

#include "liboculus/SimplePingResult.h"

namespace serdp_common {

  void drawSonar( const std::shared_ptr<liboculus::SimplePingResult> &ping, cv::Mat &mat );
  void drawSonar( const liboculus::SimplePingResult &ping, cv::Mat &mat );


}
