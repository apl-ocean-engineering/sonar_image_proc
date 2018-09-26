
#include <ctime>

#include "libg3logger/g3logger.h"

#include "serdprecorder/VideoRecorder.h"


using namespace libvideoencoder;

namespace serdprecorder {


VideoRecorder::VideoRecorder( const fs::path &outputDir )
  : _outputDir( outputDir ),
    _encoder( new Encoder("mov", AV_CODEC_ID_PRORES) ),
    _writer(nullptr)
  {;}


  VideoRecorder::~VideoRecorder()
  {
    if( _writer ) _writer.reset();
    if( _encoder ) _encoder.reset();
  }


  bool VideoRecorder::open( int width, int height, int numStreams ) {
    auto filename = makeFilename();

    _writer.reset( _encoder->makeWriter( width, height, numStreams ) );
    CHECK( _writer != nullptr );

    LOG(INFO) << "Opening video file " << filename;

    _writer->open(filename.string());

    return true;
  }


  bool VideoRecorder::close() {
    _writer.reset();
  }



  fs::path VideoRecorder::makeFilename() {

    std::time_t t = std::time(nullptr);

    char mbstr[100];
    std::strftime(mbstr, sizeof(mbstr), "vid_%Y%m%d_%H%M%S.mov", std::localtime(&t));

    // TODO.  Test if file is existing

    return fs::path(_outputDir) += mbstr;
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
