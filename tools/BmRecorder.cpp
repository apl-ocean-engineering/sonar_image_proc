
// TODO:   Reduce the DRY

#include <string>
#include <thread>
using namespace std;

#include <signal.h>

#include <CLI/CLI.hpp>

#include <libg3logger/g3logger.h>

#include "libblackmagic/DeckLink.h"
using namespace libblackmagic;

#include "libbmsdi/helpers.h"

bool keepGoing = true;


void signal_handler( int sig )
{
	LOG(INFO) << "Signal handler: " << sig;

	switch( sig ) {
		case SIGINT:
				keepGoing = false;
				break;
		default:
				keepGoing = false;
				break;
	}
}

static void processKbInput( char c, DeckLink &decklink ) {

	shared_ptr<SharedBMSDIBuffer> sdiBuffer( decklink.outputHandler().sdiProtocolBuffer() );

	switch(c) {
		case 'f':
					// Send absolute focus value
					LOG(INFO) << "Sending instantaneous autofocus to camera";
					{
						SharedBMSDIBuffer::lock_guard lock( sdiBuffer->writeMutex() );
						bmAddInstantaneousAutofocus( sdiBuffer->buffer, 1 );
					}
					break;
		 case '[':
					// Send positive focus increment
					LOG(INFO) << "Sending focus increment to camera";
					{
						SharedBMSDIBuffer::lock_guard lock( sdiBuffer->writeMutex() );
						bmAddFocusOffset( sdiBuffer->buffer, 1, 0.1 );
					}
					break;
			case ']':
					// Send negative focus increment
					LOG(INFO) << "Sending focus decrement to camera";
					{
						SharedBMSDIBuffer::lock_guard lock( sdiBuffer->writeMutex() );
						bmAddFocusOffset( sdiBuffer->buffer, 1, -0.1 );
					}
					break;

			case ';':
 					// Send positive focus increment
 					LOG(INFO) << "Sending aperture increment to camera";
 					{
 						SharedBMSDIBuffer::lock_guard lock( sdiBuffer->writeMutex() );
 						bmAddOrdinalApertureOffset( sdiBuffer->buffer, 1, 1 );
 					}
 					break;
 			case '\'':
 					// Send negative focus increment
 					LOG(INFO) << "Sending aperture decrement to camera";
 					{
 						SharedBMSDIBuffer::lock_guard lock( sdiBuffer->writeMutex() );
						bmAddOrdinalApertureOffset( sdiBuffer->buffer, 1, -1 );
 					}
 					break;


		case 's':
				// Toggle between reference sources
				static uint8_t ref = 0;
				LOG(INFO) << "Sending reference " << (int)ref;
				LOG(INFO) << "Sending instantaneous autofocus to camera";
				{
					SharedBMSDIBuffer::lock_guard lock( sdiBuffer->writeMutex() );
					bmAddReferenceSource( sdiBuffer->buffer, 1, ref );
				}

				//decklink.queueSDIBuffer( bmReferenceSource(1,ref) );

				if( ++ref > 2 ) ref = 0;
				break;
		case 'q':
				keepGoing = false;
				break;
	}

}


using cv::Mat;

int main( int argc, char** argv )
{
	libg3log::G3Logger logger("bmRecorder");

	signal( SIGINT, signal_handler );

	CLI::App app{"Simple BlackMagic camera recorder"};

	CLI11_PARSE(app, argc, argv);

	//videoOutput.setBMSDIBuffer(sdiBuffer);

	DeckLink decklink;


	// Need to wait for initialization
//	if( decklink.initializedSync.wait_for( std::chrono::seconds(1) ) == false || !decklink.initialized() ) {
	// if( !decklink.initialize() ) {
	// 	LOG(WARNING) << "Unable to initialize DeckLink";
	// 	exit(-1);
	// }

	// std::chrono::steady_clock::time_point start( std::chrono::steady_clock::now() );
	// std::chrono::steady_clock::time_point end( start + std::chrono::seconds( duration ) );

	int count = 0, miss = 0, displayed = 0;

	if( !decklink.startStreams() ) {
			LOG(WARNING) << "Unable to start streams";
			exit(-1);
	}

	while( keepGoing ) {

		std::chrono::steady_clock::time_point loopStart( std::chrono::steady_clock::now() );
		//if( (duration > 0) && (loopStart > end) ) { keepGoing = false;  break; }

		if( decklink.grab() ) {
			cv::Mat image;
			decklink.getRawImage(0, image);

			cv::imshow("Image", image);
			LOG_IF(INFO, (displayed % 50) == 0) << "Frame #" << displayed;

			char c = cv::waitKey(1);

			++displayed;

			// Take action on character
			processKbInput( c, decklink );

	 		++count;

		} else {
			// if grab() fails
			LOG(INFO) << "unable to grab frame";
			++miss;
			std::this_thread::sleep_for(std::chrono::microseconds(1000));
		}

	}

	 //std::chrono::duration<float> dur( std::chrono::steady_clock::now()  - start );

	LOG(INFO) << "End of main loop, stopping streams...";

	decklink.stopStreams();


	// LOG(INFO) << "Recorded " << count << " frames in " <<   dur.count();
	// LOG(INFO) << " Average of " << (float)count / dur.count() << " FPS";
	// LOG(INFO) << "   " << miss << " / " << (miss+count) << " misses";
	// LOG_IF( INFO, displayed > 0 ) << "   Displayed " << displayed << " frames";



		return 0;
	}
