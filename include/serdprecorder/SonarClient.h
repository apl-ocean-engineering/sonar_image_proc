#pragma once

#include <string>
#include <thread>
#include "serdprecorder/IoServiceThread.h"

#include "liboculus/StatusRx.h"
#include "liboculus/DataRx.h"
using namespace liboculus;

#include "serdprecorder/VideoRecorder.h"

namespace serdprecorder {

  class SonarClient {
  public:

    SonarClient( const std::string &ipAddr, const shared_ptr<VideoRecorder> &recorder = shared_ptr<VideoRecorder>(nullptr) );

    ~SonarClient();

    void start();

    void stop();

  protected:

    // Runs in thread
    void run();

  private:

    std::string _ipAddr;
    std::thread _thread;

    shared_ptr<VideoRecorder> _recorder;

    bool _done;

  };
}
