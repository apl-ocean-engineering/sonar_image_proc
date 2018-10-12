
#include <memory>
#include <string>
#include <fstream>

using std::string;
using std::ifstream;

#include <libg3logger/g3logger.h>
#include <CLI/CLI.hpp>

//#include <opencv2/highgui.hpp>

#include "liboculus/SimplePingResult.h"
using namespace liboculus;

#include "gpmf-parser/GPMF_parser.h"

void PrintGPMF(GPMF_stream *ms);


int main(int argc, char *argv[]) {

  libg3logger::G3Logger logger("gpmf_parser");

  CLI::App app{"Simple Oculus Sonar app"};

  int verbosity = 0;
  app.add_flag("-v", verbosity, "Additional output (use -vv for even more!)");

  string inputFilename("");
  app.add_option("input", inputFilename, "Filename to read sonar data from.");

  CLI11_PARSE(app, argc, argv);

  if( verbosity == 1 ) {
    logger.stderrHandle->call( &ColorStderrSink::setThreshold, INFO );
  } else if (verbosity > 1 ) {
    logger.stderrHandle->call( &ColorStderrSink::setThreshold, DEBUG );
  }

  // Open to the end to read the file length
  ifstream input( inputFilename, std::ios::binary  | std::ios::ate );
  if( !input.is_open() ) {
    LOG(WARNING) << "Could not open file " << inputFilename;
    exit(-1);
  }

  auto size = input.tellg();

  LOG(WARNING) << "Opening: " << inputFilename << " of size " << size;
  CHECK( size > 0 ) << "Got a bad size " << size;

  std::string data(size, '\0');
  input.seekg(0);
  input.read( &data[0], size );

  GPMF_stream gs;
  auto err = GPMF_Init( &gs, (uint32_t *)data.c_str(), size );

  CHECK( err == GPMF_OK ) << "Error while initializing GPMF: " << err;

  do {
    PrintGPMF(&gs);  // printf current GPMF KLV

    if( GPMF_Key(&gs) == MAKEID('O','C','U','S') ) {
      LOG(INFO) << "Found " <<  GPMF_RawDataSize(&gs) << " bytes of sonar data";

      char *data = (char *)GPMF_RawData(&gs);
      CHECK(data != nullptr);

      // LOG(INFO) << "Data: " << std::hex << (uint32_t)data[0] << " " << (uint32_t)data[1] << " " << (uint32_t)data[2] << " " << (uint32_t)data[3];
      // LOG(INFO) << "Data: " << std::hex << (uint32_t)data[4] << " " << (uint32_t)data[5] << " " << (uint32_t)data[6] << " " << (uint32_t)data[7];
      // LOG(INFO) << "Offset: " << std::hex << gs.pos;

      MessageHeader header( data );

      if( header.validate() ) {

        SimplePingResult ping( (char *)GPMF_RawData(&gs) );

        LOG(INFO) << "   --> Header is valid";
        ping.validate();
      }
    }

  } while (GPMF_OK == GPMF_Next(&gs, GPMF_RECURSE_LEVELS));


  exit(0);

	// int32_t ret = GPMF_OK;
	// GPMF_stream metadata_stream, *ms = &metadata_stream;
	// double metadatalength;
	// uint32_t *payload = NULL; //buffer to store GPMF samples from the MP4.
  //
  //
	// // get file return data
	// if (argc != 2)
	// {
	// 	printf("usage: %s <file_with_GPMF>\n", argv[0]);
	// 	return -1;
	// }
  //
  // printf("payload_size = %d\n", payload_size);
  //
  // //Using the GPMF_Parser, output some of the contents
  // GPMF_stream gs;
  // if (GPMF_OK == GPMF_Init(&gs, payload, payload_size))
  // {
  //   GPMF_ResetState(&gs);
  //   do
  //   {
  //     extern void PrintGPMF(GPMF_stream *);
  //     PrintGPMF(&gs);  // printf current GPMF KLV
  //   } while (GPMF_OK == GPMF_Next(&gs, GPMF_RECURSE_LEVELS));
  // }
  // printf("\n");

}
