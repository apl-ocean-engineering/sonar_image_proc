//#include "SonarData.h"
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include <vector>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "liboculus/DataRx.h"
#include "liboculus/IoServiceThread.h"
#include "liboculus/SonarPlayer.h"
#include "liboculus/StatusRx.h"

namespace serdp_common {
class PingDecoder {

public:
  struct SonarData {
  public:
    SonarData()
        : timestamp(0.0), frequency(-1.0), bearings(nullptr), ranges(nullptr),
          intensities(nullptr) {
      ;
    }
    float timestamp;
    float frequency;
    float *bearings;
    float *ranges;
    float *intensities;
  };
  SonarData pingPlayback(std::shared_ptr<liboculus::SimplePingResult> ping);

  SonarData sonarData;
};
} // namespace serdp_common
