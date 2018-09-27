#pragma once

#include <memory>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "libvideoencoder/VideoEncoder.h"


namespace serdprecorder {

  class VideoRecorder {
  public:

    VideoRecorder( const fs::path &outputDir );
    ~VideoRecorder();

    bool open( int width, int height, float frameRate, int numStreams = 1);

    bool close();

    bool isRecording() const { return bool(_writer != nullptr) && _isReady; }

    bool addFrame( AVFrame *frame, unsigned int stream = 0 );

    fs::path makeFilename();

  protected:

    int _frameNum;

    fs::path _outputDir;
    bool _isReady;

    std::shared_ptr<libvideoencoder::Encoder> _encoder;
    std::shared_ptr<libvideoencoder::VideoWriter> _writer;

  };

}
