
#include <ctime>
#include <memory>

#include "libg3logger/g3logger.h"

#include "serdprecorder/VideoRecorder.h"

#include "gpmf-write/GPMF_writer.h"

using namespace libvideoencoder;

namespace serdprecorder {

  using namespace std;

  #define GPMF_DEVICE_ID_OCULUS_SONAR  0xAA000001
  static char SonarName[] = "OculusSonar";

static char FileExtension[] = "mov";

  //=================================================================

  Recorder::Recorder()
  {;}

  Recorder::~Recorder()
  {;}

  //=================================================================

  const size_t BufferSize = 10 * 1024 * 1024;

  GPMFRecorder::GPMFRecorder( )
    : Recorder(),
      _scratch( new uint32_t[BufferSize] ),
      _out(),
      _gpmfHandle( GPMFWriteServiceInit() ),
      _sonarHandle( GPMFWriteStreamOpen(_gpmfHandle, GPMF_CHANNEL_TIMED, GPMF_DEVICE_ID_OCULUS_SONAR, SonarName, NULL, BufferSize) )
    {
      CHECK( _gpmfHandle ) << "Unable to initialize GPMF Write Service";
      CHECK( _sonarHandle ) << "Unable to initialize GPMF stream for sonar";

      initGPMF();
    }

  GPMFRecorder::~GPMFRecorder()
  {
    GPMFWriteStreamClose(_sonarHandle);
    GPMFWriteServiceClose(_gpmfHandle);
  }

  bool GPMFRecorder::open( const std::string &filename ) {

    _out.open( filename, ios_base::binary | ios_base::out );

    if( !_out.is_open() ) return false;

    flushGPMF();

    return true;
  }

  void GPMFRecorder::close() {
    _out.close();
  }

  void GPMFRecorder::initGPMF()
  {
    GPMFWriteSetScratchBuffer( _gpmfHandle, _scratch.get(), BufferSize );

    const char name[]="Oculus MB1200d";
    GPMFWriteStreamStore(_sonarHandle, GPMF_KEY_STREAM_NAME, GPMF_TYPE_STRING_ASCII, strlen(name), 1, (void *)name, GPMF_FLAGS_STICKY);
  }

  void GPMFRecorder::flushGPMF()
  {
    // Flush any stale data before starting video capture.
    uint32_t *payload, payload_size;
    GPMFWriteGetPayload(_gpmfHandle, GPMF_CHANNEL_TIMED, _scratch.get(), BufferSize, &payload, &payload_size);
  }

  size_t GPMFRecorder::writeSonar( const std::shared_ptr<liboculus::SimplePingResult> &ping, uint32_t **buffer, size_t bufferSize )
  {
    {
      // Mark as big endian so it doesn't try to byte-swap the data.
      LOG(INFO) << "Writing " << (ping->dataSize() >> 2) << " dwords of sonar";
      auto err = GPMFWriteStreamStore(_sonarHandle, STR2FOURCC("OCUS"), GPMF_TYPE_UNSIGNED_LONG,
                                          4, (ping->dataSize() >> 2), ping->data(), GPMF_FLAGS_BIG_ENDIAN);
      if( err != GPMF_ERROR_OK ) {
        LOG(WARNING) << "Error writing to GPMF store";
        return 0;
      }
    }

    // Estimate buffer size
    size_t estSize = GPMFWriteEstimateBufferSize( _gpmfHandle, GPMF_CHANNEL_TIMED, 0 ) + 8192;

    if( estSize > bufferSize ) {
      *buffer = (uint32_t *)av_malloc( estSize );
      bufferSize = estSize;
    }

    uint32_t *payload, payloadSize;
    GPMFWriteGetPayload(_gpmfHandle, GPMF_CHANNEL_TIMED, *buffer, bufferSize, &payload, &payloadSize);

    CHECK( payload == *buffer );

    LOG(DEBUG) << "Estimated GPMF size (in bytes) " << estSize << " ; actual payload size (in bytes) " << payloadSize;

    return payloadSize;
  }


  bool GPMFRecorder::addSonar( const std::shared_ptr<liboculus::SimplePingResult> &ping ) {
    if( !_out.is_open() ) return false;

    uint32_t *buffer = nullptr;
    auto bufferSize = writeSonar( ping, &buffer, 0 );
    if( bufferSize == 0 ) return false;

    _out.write( (const char *)buffer, bufferSize );

    if( buffer != nullptr ) av_free(buffer);

    return true;
  }


  //=================================================================

  VideoRecorder::VideoRecorder( )
    : GPMFRecorder(),
      _frameNum(0),
      _doSonar( false ),
      _sonarTrack( -1 ),
      _outputDir( "/tmp/" ),
      _isReady( false ),
      _pending(0),
      _mutex(),
      _encoder( new Encoder(FileExtension, AV_CODEC_ID_PRORES) ),
      _writer(nullptr)
  {
    ;
  }

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
    _sonarWritten = 0;
    _startTime = std::chrono::system_clock::now();

    flushGPMF();

    _isReady = true;
    {
      std::lock_guard<std::mutex> lock(_mutex);
      _pending = 0;
    }
    return true;
  }


  void VideoRecorder::close() {

    while(true) {
      std::lock_guard<std::mutex> lock(_mutex);
      if( _pending == 0 ) break;
    }

    LOG(INFO) << "Closing video with " << _frameNum << " frames";
    LOG_IF(INFO, _doSonar) << "     Wrote " << _sonarWritten << " frames of sonar";

    _isReady = false;
    _frameNum = 0;
    _writer.reset();
  }


  bool VideoRecorder::addMats( std::vector<cv::Mat> &mats ) {
    if( !isRecording() ) return false;

    std::deque< std::shared_ptr<std::thread> > recordThreads;

    // Video corruption when this encoding is split into two threads.
    for( unsigned int i=0; i < mats.size(); ++i ) {
       recordThreads.push_back( shared_ptr<std::thread>( new std::thread( &VideoRecorder::addMat, this, mats[i], i ) ) );
    }

    // // Wait for all recorder threads
    for( auto thread : recordThreads ) thread->join();

    advanceFrame();

    return true;
  }


  bool VideoRecorder::addMat( cv::Mat image, unsigned int stream ) {
      if( !isRecording() ) return false;

    if( !_writer ) return false;

    {
      std::lock_guard<std::mutex> lock(_mutex);
      ++_pending;
    }

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

    res = _writer->addFrame( frame, _frameNum, stream );

    av_frame_free( &frame );

    {
      std::lock_guard<std::mutex> lock(_mutex);
      --_pending;
    }

    return res;
  }

  bool VideoRecorder::addSonar( const std::shared_ptr<liboculus::SimplePingResult> &ping ) {
    if( !isRecording() ) return false;

    if( !_writer ) return false;
    if( _sonarTrack < 0 ) return false;

    {
      std::lock_guard<std::mutex> lock(_mutex);
      ++_pending;
    }

    {
      uint32_t *buffer;

      auto payloadSize = writeSonar( ping, &buffer, 0 );
      if( payloadSize == 0) {
        LOG(WARNING) << "Failed to write sonar!";
        {
          std::lock_guard<std::mutex> lock(_mutex);
          --_pending;
        }
        return false;
      }

      // And data track
      AVPacket *pkt = av_packet_alloc();
      av_packet_from_data(pkt, (uint8_t *)buffer, payloadSize );

      pkt->stream_index = _sonarTrack;

      std::chrono::microseconds timePt =  std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - _startTime);

      pkt->dts = timePt.count();
      pkt->pts = pkt->dts;

      LOG(WARNING) << "   packet->pts " << pkt->pts;

      ++_sonarWritten;
      _writer->addPacket( pkt );
    }

    {
      std::lock_guard<std::mutex> lock(_mutex);
      --_pending;
    }
    return true;
  }


  fs::path VideoRecorder::makeFilename() {

    std::time_t t = std::time(nullptr);

    char mbstr[100];
    std::strftime(mbstr, sizeof(mbstr), "vid_%Y%m%d_%H%M%S", std::localtime(&t));

    // TODO.  Test if file is existing

    strcat( mbstr, ".");
    strcat( mbstr, FileExtension);

    return fs::path(_outputDir) /= mbstr;
  }



}
