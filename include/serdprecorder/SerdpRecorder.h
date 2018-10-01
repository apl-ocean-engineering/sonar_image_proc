#pragma once


#include <CLI/CLI.hpp>

#include <libg3logger/g3logger.h>

#include "libblackmagic/DeckLink.h"
#include "libblackmagic/DataTypes.h"
using namespace libblackmagic;

#include "libbmsdi/helpers.h"

#include "libvideoencoder/VideoEncoder.h"
using libvideoencoder::Encoder;

#include "serdprecorder/CameraState.h"
#include "serdprecorder/VideoRecorder.h"
#include "serdprecorder/SonarClient.h"
#include "serdprecorder/OpenCVDisplay.h"


using cv::Mat;


namespace serdprecorder {

  class SerdpRecorder {
  public:

    SerdpRecorder();

    bool keepGoing( bool kg ) { return _keepGoing = kg; }

    int run( int argc, char **argv );

    void handleKey( const char c );

  protected:

    bool _keepGoing;

    DeckLink _deckLink;
    CameraState _camState;
    std::shared_ptr<VideoRecorder> _recorder;
    shared_ptr<SonarClient> _sonar;


  };

}
