
#include <string>
#include <iostream>

#include "VideoEncoder.h"


using Encoder::VideoEncoder;


int main( int argc, char **argv ) {


  VideoEncoder encoder;


  auto result = encoder.InitFile("test.mov", "auto");

  if( result ) {

  } else {
    std::cerr << "Unable to initialize encoder." << std::endl;;
  }


  encoder.Finish();


  return 0;
}
