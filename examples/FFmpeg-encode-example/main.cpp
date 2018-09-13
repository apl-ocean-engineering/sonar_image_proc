
#include <string>
#include <iostream>

#include "VideoEncoder.h"

#include "Settings.h"

using Encoder::VideoEncoder;


int main( int argc, char **argv ) {


  VideoEncoder encoder;


  auto result = encoder.InitFile("/tmp/test.mov", "auto");

  if( result ) {

    AVFrame *frame = av_frame_alloc();   ///avcodec_alloc_frame();
    uint8_t *picture_buf = NULL;
    int size;

  	if ( !frame )
  	{
  		printf("Cannot create frame\n");
  		exit(-1);
  	}

    frame->width = W_VIDEO;
    frame->height = H_VIDEO;
    frame->format = AV_PIX_FMT_RGB24;

  	auto res = av_frame_get_buffer(frame, 0);

  	if (res < 0) {
  		av_frame_free( &frame);
  		printf("Cannot allocate buffer\n");
  		exit(-1);
  	}

    const int numFrames = 1;

    for( int frameNum = 0; frameNum < numFrames; ++frameNum ) {
      encoder.AddFrame( frame, nullptr, 0 );
    }

    av_frame_free( &frame );

  } else {
    std::cerr << "Unable to initialize encoder." << std::endl;;
  }

  encoder.Finish();


  return 0;
}
