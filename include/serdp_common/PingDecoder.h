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

#include "DataStructures.h"

namespace serdp_common {

class PingDecoder {

public:
  std::shared_ptr<SonarData>
  pingPlayback(std::shared_ptr<liboculus::SimplePingResult> ping);

  // std::shared_ptr<SonarData> sonarData;
};
} // namespace serdp_common
