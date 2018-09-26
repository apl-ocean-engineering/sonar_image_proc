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

    bool open( int width, int height, int numStreams );

    bool close();

    bool isRecording() const { return bool(_writer != nullptr); }

    fs::path makeFilename();

  protected:

    fs::path _outputDir;

    std::shared_ptr<libvideoencoder::Encoder> _encoder;
    std::shared_ptr<libvideoencoder::VideoWriter> _writer;

  };

}
