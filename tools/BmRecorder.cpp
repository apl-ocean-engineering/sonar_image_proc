
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

	switch(c) {
		case 'f':
				// Send focus
				LOG(INFO) << "Sending instantaneous autofocus to camera";
				//decklink.queueSDIBuffer( bmInstantaneousAutofocus(1) );
				break;
		case 's':
				// Toggle between reference sources
				static uint8_t ref = 0;
				LOG(INFO) << "Sending reference " << (int)ref;

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


	std::shared_ptr<BMSDIBuffer> sdiBuffer;



	// Need to wait for initialization
//	if( decklink.initializedSync.wait_for( std::chrono::seconds(1) ) == false || !decklink.initialized() ) {
	// if( !decklink.initialize() ) {
	// 	LOG(WARNING) << "Unable to initialize DeckLink";
	// 	exit(-1);
	// }

	// std::chrono::steady_clock::time_point start( std::chrono::steady_clock::now() );
	// std::chrono::steady_clock::time_point end( start + std::chrono::seconds( duration ) );

	int count = 0, miss = 0, displayed = 0;
	bool logOnce = true;

	if( !decklink.startStreams() ) {
			LOG(WARNING) << "Unable to startStreams";
			exit(-1);
	}

	while( keepGoing ) {

		if( count > 0 && (count % 100)==0 ) {
			LOG_IF(INFO, logOnce) << count << " frames";
			logOnce = false;
		} else {
			logOnce = true;
		}

		std::chrono::steady_clock::time_point loopStart( std::chrono::steady_clock::now() );
		//if( (duration > 0) && (loopStart > end) ) { keepGoing = false;  break; }

		//if( decklink.grab() ) {
		if( true ) {
			cv::Mat image;
			//decklink.getImage(0, image);

			cv::imshow("Image", image);
			char c = cv::waitKey(1);

				++displayed;

			// Take action on character

			processKbInput( c, decklink );

	 		++count;

		} else {
			// if grab() fails
			LOG(INFO) << "unable to grab frame";
			++miss;
			std::this_thread::sleep_for(std::chrono::microseconds(100));
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
