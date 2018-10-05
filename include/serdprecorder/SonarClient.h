#pragma once

#include <string>
#include <thread>
#include "serdprecorder/IoServiceThread.h"

#include "liboculus/StatusRx.h"
#include "liboculus/DataRx.h"
using namespace liboculus;

#include "serdprecorder/VideoRecorder.h"
#include "serdprecorder/OpenCVDisplay.h"

namespace serdprecorder {

  class SonarClient {
  public:

    SonarClient( const std::string &ipAddr,
                  const shared_ptr<Recorder> &recorder = shared_ptr<Recorder>(nullptr),
                  const shared_ptr<OpenCVDisplay> &display = shared_ptr<OpenCVDisplay>(nullptr) );

    ~SonarClient();

    void start();

    void stop();

  protected:

    // Runs in thread
    void run();

  private:

    std::string _ipAddr;
    std::thread _thread;

    shared_ptr<Recorder> _recorder;
    shared_ptr<OpenCVDisplay> _display;

    bool _done;

  };
}
