#pragma once

#include <memory>
#include <fstream>
#include <chrono>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <opencv2/opencv.hpp>

#include "libvideoencoder/VideoEncoder.h"

#include "liboculus/SimplePingResult.h"


namespace serdprecorder {

  class Recorder {
  public:
    Recorder();
    virtual ~Recorder();

    // bool setDoSonar( bool d )                   { return _doSonar = d; }
    // void setOutputDir( const std::string &dir ) { _outputDir = dir; }
    //
    // virtual bool open( int width, int height, float frameRate, int numStreams = 1);
    // void close();

    virtual bool addMats( std::vector<cv::Mat> &mats ) = 0;

    virtual bool addSonar( const std::shared_ptr<liboculus::SimplePingResult> &ping ) = 0;

  };

  //==


    //==

    class GPMFRecorder : public Recorder {
    public:

      GPMFRecorder();
      //VideoRecorder( const fs::path &outputDir, bool doSonar = false );
      virtual ~GPMFRecorder();

      bool open( const std::string &filename );
      void close();

      bool isRecording() const { return _out.is_open(); }

      virtual bool addMats( std::vector<cv::Mat> &mats ) { return false; }
      virtual bool addSonar( const std::shared_ptr<liboculus::SimplePingResult> &ping );

    protected:

      void initGPMF();
      void flushGPMF();
      size_t writeSonar( const std::shared_ptr<liboculus::SimplePingResult> &ping, uint32_t **buffer, size_t bufferSize );


      std::unique_ptr<uint32_t> _scratch;

      std::ofstream _out;

      size_t _gpmfHandle;
      size_t _sonarHandle;

    };

 //==

  class VideoRecorder : public GPMFRecorder {
  public:

    VideoRecorder();
    //VideoRecorder( const fs::path &outputDir, bool doSonar = false );
    virtual ~VideoRecorder();

    bool setDoSonar( bool d )                   { return _doSonar = d; }
    void setOutputDir( const std::string &dir ) { _outputDir = dir; }

    bool open( int width, int height, float frameRate, int numStreams = 1);

    void close();

    bool isRecording() const { return bool(_writer != nullptr) && _isReady; }

    virtual bool addMats( std::vector<cv::Mat> &mats );

    bool addMat( cv::Mat image, unsigned int stream = 0  );
    //bool addFrame( AVFrame *frame, unsigned int stream = 0 );

    virtual bool addSonar( const std::shared_ptr<liboculus::SimplePingResult> &ping );

    void advanceFrame() { ++_frameNum; }

    fs::path makeFilename();

  protected:

    int _frameNum;

    unsigned int _sonarWritten;

    bool _doSonar;
    int _sonarTrack;

    fs::path _outputDir;
    bool _isReady;

    unsigned int _pending;
    std::mutex _mutex;

    std::chrono::time_point< std::chrono::system_clock > _startTime;

    std::shared_ptr<libvideoencoder::Encoder> _encoder;
    std::shared_ptr<libvideoencoder::VideoWriter> _writer;


  };


}
