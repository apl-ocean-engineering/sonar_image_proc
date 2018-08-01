
// TODO:   Reduce the DRY

#include <string>
#include <thread>
using namespace std;

#include <signal.h>

#include <CLI/CLI.hpp>

#include <libg3logger/g3logger.h>

#include "libblackmagic/DeckLink.h"
#include "libblackmagic/DataTypes.h"
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

const int CamNum = 1;



static void processKbInput( char c, DeckLink &decklink ) {

	shared_ptr<SharedBMSDIBuffer> sdiBuffer( decklink.output().sdiProtocolBuffer() );

	SDIBufferGuard guard( sdiBuffer );

	switch(c) {
		case 'f':
					// Send absolute focus value
					LOG(INFO) << "Sending instantaneous autofocus to camera";
					guard( []( BMSDIBuffer *buffer ){ bmAddInstantaneousAutofocus( buffer, CamNum ); });
					break;
		 case '[':
					// Send positive focus increment
					LOG(INFO) << "Sending focus increment to camera";
					guard( []( BMSDIBuffer *buffer ){	bmAddFocusOffset( buffer, CamNum, 0.05 ); });
					break;
			case ']':
					// Send negative focus increment
					LOG(INFO) << "Sending focus decrement to camera";
					guard( []( BMSDIBuffer *buffer ){ bmAddFocusOffset( buffer, CamNum, -0.05 ); });
					break;

			//=== Aperture increment/decrement ===
			case ';':
 					// Send positive aperture increment
 					LOG(INFO) << "Sending aperture increment to camera";
 					guard( []( BMSDIBuffer *buffer ){	bmAddOrdinalApertureOffset( buffer, CamNum, 1 ); });
 					break;
 			case '\'':
 					// Send negative aperture decrement
 					LOG(INFO) << "Sending aperture decrement to camera";
					guard( []( BMSDIBuffer *buffer ){	bmAddOrdinalApertureOffset( buffer, CamNum, -1 ); });
 					break;

			//=== Shutter increment/decrement ===
			case '.':
 					LOG(INFO) << "Sending shutter increment to camera";
					guard( []( BMSDIBuffer *buffer ){	bmAddOrdinalShutterOffset( buffer, CamNum, 1 ); });
 					break;
 			case '/':
 					LOG(INFO) << "Sending shutter decrement to camera";
					guard( []( BMSDIBuffer *buffer ){	bmAddOrdinalShutterOffset( buffer, CamNum, -1 ); });
 					break;

			//=== Gain increment/decrement ===
			case 'z':
 					LOG(INFO) << "Sending gain increment to camera";
					guard( []( BMSDIBuffer *buffer ){	bmAddSensorGainOffset( buffer, CamNum, 1 ); });
 					break;
 			case 'x':
 					LOG(INFO) << "Sending gain decrement to camera";
					guard( []( BMSDIBuffer *buffer ){	bmAddSensorGainOffset( buffer, CamNum, -1 ); });
 					break;

			//== Increment/decrement white balance
			case 'w':
					LOG(INFO) << "Auto white balance";
					guard( []( BMSDIBuffer *buffer ){	bmAddAutoWhiteBalance( buffer, CamNum ); });
					break;

			case 'e':
					LOG(INFO) << "Restore white balance";
					guard( []( BMSDIBuffer *buffer ){	bmAddRestoreWhiteBalance( buffer, CamNum ); });
					break;

			case 'r':
					LOG(INFO) << "Sending decrement to white balance";
					guard( []( BMSDIBuffer *buffer ){	bmAddWhiteBalanceOffset( buffer, CamNum, -1000, 0 ); });
					break;

			case 't':
					LOG(INFO) << "Sending increment to white balance";
					guard( []( BMSDIBuffer *buffer ){	bmAddWhiteBalanceOffset( buffer, CamNum, 1000, 0 ); });
					break;

		case 's':
				// Toggle between reference sources
				LOG(INFO) << "Switching reference source";
				guard( []( BMSDIBuffer *buffer ){	bmAddReferenceSourceOffset( buffer, CamNum, 1 ); });
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
	logger.stderrHandle->call( &ColorStderrSink::setThreshold, DEBUG );

	signal( SIGINT, signal_handler );

	CLI::App app{"Simple BlackMagic camera recorder"};

	bool do3D = false;
	app.add_flag("--do-3d",do3D, "Enable 3D modes");

	bool noDisplay = false;
	app.add_flag("--no-display,-x", noDisplay, "Disable display");

	string desiredModeString = "auto";
	app.add_option("--mode,-m", desiredModeString, "Desired mode");

	bool doConfigCamera = false;
	app.add_flag("--config-camera,-c", doConfigCamera, "If enabled, send initialization info to the cameras");

	bool doListCards = false;
	app.add_flag("--list-cards", doListCards, "List Decklink cards in the system then exit");

	bool doListInputModes = false;
	app.add_flag("--list-input-modes", doListInputModes, "List Input modes then exit");

	int stopAfter = -1;
	app.add_option("--stop-after", stopAfter, "Stop after N frames");

	CLI11_PARSE(app, argc, argv);

	BMDDisplayMode mode = stringToDisplayMode( desiredModeString );
	if( mode == bmdModeUnknown ) {
		LOG(WARNING) << "Didn't understand mode \"" << desiredModeString << "\"";
		return -1;
	} else if ( mode == bmdModeDetect ) {
		LOG(WARNING) << "Will attempt input format detection";
	}

	// Help string
	cout << "Commands" << endl;
	cout << "    q       quit" << endl;
	cout << "   [ ]     Adjust focus" << endl;
	cout << "    f      Set autofocus" << endl;
	cout << "   ; '     Adjust aperture" << endl;
	cout << "   . /     Adjust shutter speed" << endl;
	cout << "   z x     Adjust sensor gain" << endl;
	cout << "    s      Cycle through reference sources" << endl;

	DeckLink deckLink;

	// Handle the one-off commands
	if( doListCards || doListInputModes ) {
			if(doListCards) deckLink.listCards();
			if(doListInputModes) deckLink.listInputModes();
		return 0;
	}

	//  Input should always auto-detect
	deckLink.input().config().setMode( bmdModeDetect );
	deckLink.input().config().set3D( do3D );
		/* code */

	// CHECK( deckLink.createVideoOutput(bmdModeHD1080p2997) ) << "Unable to create VideoOutput";
	// CHECK( deckLink.createVideoInput(bmdMode4K2160p2997) ) << "Unable to create VideoInput";

	// Need to wait for initialization
//	if( decklink.initializedSync.wait_for( std::chrono::seconds(1) ) == false || !decklink.initialized() ) {
	// if( !decklink.initialize() ) {
	// 	LOG(WARNING) << "Unable to initialize DeckLink";
	// 	exit(-1);
	// }

	// std::chrono::steady_clock::time_point start( std::chrono::steady_clock::now() );
	// std::chrono::steady_clock::time_point end( start + std::chrono::seconds( duration ) );

	int count = 0, miss = 0, displayed = 0;

	if( !deckLink.startStreams() ) {
			LOG(WARNING) << "Unable to start streams";
			exit(-1);
	}


		if ( doConfigCamera ) {
			LOG(INFO) << "Sending configuration to cameras";

			// Be careful not to exceed 255 byte buffer length
			SDIBufferGuard guard( deckLink.output().sdiProtocolBuffer() );
			guard( [mode]( BMSDIBuffer *buffer ) {

				bmAddOrdinalAperture( buffer, CamNum, 0 );
				bmAddSensorGain( buffer, CamNum, 8 );
				bmAddReferenceSource( buffer, CamNum, BM_REF_SOURCE_PROGRAM );
				bmAddAutoWhiteBalance( buffer, CamNum );

				if(mode != bmdModeDetect) {
					LOG(INFO) << "Setting video mode";
					bmAddVideoMode( buffer, CamNum, bmdModeHD1080p2997 );
				}

			});
		}

	while( keepGoing ) {

		std::chrono::steady_clock::time_point loopStart( std::chrono::steady_clock::now() );
		//if( (duration > 0) && (loopStart > end) ) { keepGoing = false;  break; }

		int numImages = 0;
		if( (numImages = deckLink.input().grab()) > 0 ) {

			LOG(INFO) << "Got " << numImages << " images";

			std::array<cv::Mat,2> images;

			for( auto i=0; i < count && i < images.size(); ++i ) {
				deckLink.input().getRawImage(i, images[i]);
			}

			if( !noDisplay ) {

				if( numImages == 1 ) {
					cv::imshow("Image", images[0]);
				} else if ( numImages == 2 ) {

					cv::Mat composite( cv::Size( images[0].size().width + images[0].size().width,
					std::max(images[0].size().height, images[1].size().height )), images[0].type() );

					cv::Mat leftROI( composite, cv::Rect(0,0,images[0].size().width,images[0].size().height) );
					images[0].copyTo( leftROI );

					cv::Mat rightROI( composite, cv::Rect(images[0].size().width, 0, images[1].size().width, images[1].size().height) );
					images[1].copyTo( rightROI );

					cv::imshow("Composite", composite );

				}

				LOG_IF(INFO, (displayed % 50) == 0) << "Frame #" << displayed;
				char c = cv::waitKey(1);

				++displayed;

				// Take action on character
				processKbInput( c, deckLink );
			}


		} else {
			// if grab() fails
			LOG(INFO) << "unable to grab frame";
			++miss;
			std::this_thread::sleep_for(std::chrono::microseconds(1000));
		}

		++count;

		if((stopAfter > 0) && (count >= stopAfter)) keepGoing = false;

	}

	 //std::chrono::duration<float> dur( std::chrono::steady_clock::now()  - start );

	LOG(INFO) << "End of main loop, stopping streams...";

	deckLink.stopStreams();


	// LOG(INFO) << "Recorded " << count << " frames in " <<   dur.count();
	// LOG(INFO) << " Average of " << (float)count / dur.count() << " FPS";
	// LOG(INFO) << "   " << miss << " / " << (miss+count) << " misses";
	// LOG_IF( INFO, displayed > 0 ) << "   Displayed " << displayed << " frames";



		return 0;
	}
