
#include <ctime>

#include "libg3logger/g3logger.h"

#include "serdprecorder/VideoRecorder.h"

#include "gpmf-write/GPMF_writer.h"

using namespace libvideoencoder;

namespace serdprecorder {

  #define GPMF_DEVICE_ID_OCULUS_SONAR  0xAA000001
  static char SonarName[] = "OculusSonar";

  VideoRecorder::VideoRecorder( )
    : _frameNum(0),
      _doSonar( false ),
      _sonarTrack( -1 ),
      _outputDir( "/tmp/" ),
      _isReady( false ),
      _encoder( new Encoder("mov", AV_CODEC_ID_PRORES) ),
      _writer(nullptr),
      _gpmfHandle( GPMFWriteServiceInit() ),
      _sonarHandle( GPMFWriteStreamOpen(_gpmfHandle, GPMF_CHANNEL_TIMED, GPMF_DEVICE_ID_OCULUS_SONAR, SonarName, NULL, 0) )
    {
      CHECK( _gpmfHandle ) << "Unable to initialize GPMF Write Service";
      CHECK( _sonarHandle ) << "Unable to initialize GPMF stream for sonar";

      initGPMF();
    }

// VideoRecorder::VideoRecorder( const fs::path &outputDir, bool doSonar )
//   : _frameNum(0),
//     _doSonar( doSonar ),
//     _sonarTrack( -1 ),
//     _outputDir( outputDir ),
//     _isReady( false ),
//     _encoder( new Encoder("mov", AV_CODEC_ID_PRORES) ),
//     _writer(nullptr)
//   {;}


  VideoRecorder::~VideoRecorder()
  {
    if( _writer ) _writer.reset();
    if( _encoder ) _encoder.reset();
  }


  bool VideoRecorder::open( int width, int height, float frameRate, int numStreams ) {
    auto filename = makeFilename();

    _writer.reset( _encoder->makeWriter() );
    CHECK( _writer != nullptr );

    _writer->addVideoTrack( width, height, frameRate, numStreams );

    if( _doSonar )
      _sonarTrack = _writer->addDataTrack();

    LOG(INFO) << "Opening video file " << filename;

    _writer->open(filename.string());
    _frameNum = 0;

    // Flush GPMF stream
    //Flush any stale data before starting video capture.
    {
      char buffer[8192];
      uint32_t *payload, payload_size;
	    GPMFWriteGetPayload(_gpmfHandle, GPMF_CHANNEL_TIMED, (uint32_t *)buffer, sizeof(buffer), &payload, &payload_size);
    }

    _isReady = true;
    return true;
  }


  void VideoRecorder::close() {
    LOG(INFO) << "Closing video with " << _frameNum << " frames";
    _isReady = false;
    _frameNum = 0;
    _writer.reset();
  }



  bool VideoRecorder::addMat( cv::Mat image, unsigned int stream ) {
    if( !_writer ) return false;
    //Convert to AVFrame
    AVFrame *frame = av_frame_alloc();   ///avcodec_alloc_frame();
    CHECK( frame != nullptr ) << "Cannot create frame";

    auto sz = image.size();

    frame->width = sz.width;
    frame->height = sz.height;
    frame->format = AV_PIX_FMT_BGRA;
    auto res = av_frame_get_buffer(frame, 0);

    // Try this_`
    // memcpy for now
    cv::Mat frameMat( sz.height, sz.width, CV_8UC4, frame->data[0]);
    image.copyTo( frameMat );

    auto result = addFrame( frame, stream );

    av_frame_free( &frame );

    return result;
  }

  bool VideoRecorder::addFrame( AVFrame *frame, unsigned int stream ) {
    if( !_writer ) return false;

    if( !_writer->addFrame( frame, _frameNum, stream ) ) return false;

    return true;
  }

  bool VideoRecorder::addSonar( void *data, size_t sz ) {
    if( !_writer ) return false;

    if( _sonarTrack < 0 ) return false;


    {
      // Add sonar data to GPMF handle
      auto err = GPMFWriteStreamStore(_sonarHandle, STR2FOURCC("OCUS"), GPMF_TYPE_COMPLEX, 1, sz, data, GPMF_FLAGS_NONE);
			LOG_IF(WARNING,err) << "Error writing to GPMF store";
    }

    {

      // Estimate buffer size
      size_t estSize = GPMFWriteEstimateBufferSize( _gpmfHandle, GPMF_CHANNEL_TIMED, 0 );
      // Inflate estimated size?

      void *buffer = av_malloc( estSize + 8192 );

      uint32_t *payload, payloadSize;

      GPMFWriteGetPayload(_gpmfHandle, GPMF_CHANNEL_TIMED, (uint32_t *)buffer, sizeof(buffer), &payload, &payloadSize);

      // And data track
      AVPacket *pkt = av_packet_alloc();
      av_packet_from_data(pkt, (uint8_t *)buffer, payloadSize );
      //memcpy( pkt->data, data, sz );

      pkt->stream_index = _sonarTrack;
      pkt->dts = 0;
      pkt->pts = pkt->dts;

      _writer->addPacket( pkt );
    }

    return true;
  }


  fs::path VideoRecorder::makeFilename() {

    std::time_t t = std::time(nullptr);

    char mbstr[100];
    std::strftime(mbstr, sizeof(mbstr), "vid_%Y%m%d_%H%M%S.mov", std::localtime(&t));

    // TODO.  Test if file is existing

    return fs::path(_outputDir) += mbstr;
  }


  //=================================
  void VideoRecorder::initGPMF()
  {
    // Add sticky deckLinkAttributes
    const char name[]="Oculus MB1200d";
		GPMFWriteStreamStore(_sonarHandle, GPMF_KEY_STREAM_NAME, GPMF_TYPE_STRING_ASCII, strlen(name), 1, (void *)name, GPMF_FLAGS_STICKY);
  }


  // static shared_ptr<Encoder> MakeVideoEncoder( const string &outputFile, const BMDDisplayMode mode, bool do3D = false ) {
  //
  // 	auto modeStruct = decodeBMDMode( mode );
  //
  // 	shared_ptr<Encoder> videoOutput( new Encoder(modeStruct->width, modeStruct->height, modeStruct->frameRate ) );
  // 	videoOutput->InitFile( outputFile, "auto", AV_CODEC_ID_PRORES, do3D ? 2 : 1 );
  //
  // 	return videoOutput;
  // }
  //
  // static shared_ptr<Encoder> MakeVideoEncoder( const string &outputFile, const libblackmagic::InputConfig &config ) {
  // 	return MakeVideoEncoder( outputFile, config.mode(), config.do3D() );
  // }



}
