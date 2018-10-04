#pragma once

#include <memory>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <opencv2/opencv.hpp>

#include "libvideoencoder/VideoEncoder.h"


namespace serdprecorder {

  class VideoRecorder {
  public:

    VideoRecorder();
    //VideoRecorder( const fs::path &outputDir, bool doSonar = false );
    ~VideoRecorder();

    bool setDoSonar( bool d )                   { return _doSonar = d; }
    void setOutputDir( const std::string &dir ) { _outputDir = dir; }

    bool open( int width, int height, float frameRate, int numStreams = 1);

    void close();

    bool isRecording() const { return bool(_writer != nullptr) && _isReady; }

    bool addMat( cv::Mat image, unsigned int stream = 0  );
    bool addFrame( AVFrame *frame, unsigned int stream = 0 );

    bool addSonar( void *data, size_t size );

    void advanceFrame() { ++_frameNum; }

    fs::path makeFilename();

  protected:

    void initGPMF();

    int _frameNum;

    bool _doSonar;
    int _sonarTrack;

    fs::path _outputDir;
    bool _isReady;

    std::shared_ptr<libvideoencoder::Encoder> _encoder;
    std::shared_ptr<libvideoencoder::VideoWriter> _writer;

    size_t _gpmfHandle;
    size_t _sonarHandle;

  };

}
