//#include "SonarData.h"
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include <vector>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <CLI/CLI.hpp>
#include <libg3logger/g3logger.h>

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
        : frequency(-1), bearings(nullptr), ranges(nullptr),
          intensities(nullptr) {
      ;
    }
    float frequency;
    float *bearings;
    float *ranges;
    float *intensities;
  };
  std::shared_ptr<SonarData>
  pingPlayback(std::shared_ptr<liboculus::SimplePingResult> ping);

  std::shared_ptr<SonarData> sonarData;
};
} // namespace serdp_common
