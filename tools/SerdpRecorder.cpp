
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

#include "libvideoencoder/VideoEncoder.h"
using libvideoencoder::Encoder;

#include "serdprecorder/CameraState.h"
#include "serdprecorder/VideoRecorder.h"
#include "serdprecorder/SonarClient.h"
using namespace serdprecorder;


using cv::Mat;

bool keepGoing = true;

using namespace serdp_recorder;

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

DeckLink deckLink;
shared_ptr<VideoRecorder> recorder(nullptr);
shared_ptr<SonarClient> sonar(nullptr);



static void processKbInput( char c, DeckLink &decklink, CameraState &camState ) {

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
			case '\'':
					{
						// Send positive aperture increment
						auto val = camState.apertureInc();
	 					// LOG(INFO) << "Sending aperture increment " << val.ord << " , " << val.val << " , " << val.str;
						LOG(INFO) << "Set aperture to " << val;
					}
 					// guard( []( BMSDIBuffer *buffer ){	bmAddOrdinalApertureOffset( buffer, CamNum, 1 ); });
 					break;
 			case ';':
					{
						// Send negative aperture decrement
						auto val = camState.apertureDec();
						//LOG(INFO) << "Sending aperture decrement " << val.ord << " , " << val.val << " , " << val.str;
						LOG(INFO) << "Set aperture to " << val;
					// guard( []( BMSDIBuffer *buffer ){	bmAddOrdinalApertureOffset( buffer, CamNum, -1 ); });
					}
 					break;

			//=== Shutter increment/decrement ===
			case '.':
 					LOG(INFO) << "Sending shutter increment to camera";
					camState.exposureInc();
 					break;
 			case '/':
 					LOG(INFO) << "Sending shutter decrement to camera";
					camState.exposureDec();
 					break;

			//=== Gain increment/decrement ===
			case 'z':
 					LOG(INFO) << "Sending gain increment to camera";
					camState.gainInc();
 					break;
 			case 'x':
 					LOG(INFO) << "Sending gain decrement to camera";
					camState.gainDec();
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
					guard( []( BMSDIBuffer *buffer ){	bmAddWhiteBalanceOffset( buffer, CamNum, -500, 0 ); });
					break;

			case 't':
					LOG(INFO) << "Sending increment to white balance";
					guard( []( BMSDIBuffer *buffer ){	bmAddWhiteBalanceOffset( buffer, CamNum, 500, 0 ); });
					break;

			case '1':
					LOG(INFO) << "Setting camera to 1080p2997";
					// guard( [](BMSDIBuffer *buffer ){bmAddReferenceSource( buffer, CamNum, BM_REF_SOURCE_PROGRAM );});
					guard( [](BMSDIBuffer *buffer ){
						bmAddVideoMode( buffer, CamNum,bmdModeHD1080p2997 );
						bmAddReferenceSource( buffer, CamNum, BM_REF_SOURCE_PROGRAM );
						bmAddAutoExposureMode( buffer, CamNum, BM_AUTOEXPOSURE_SHUTTER );
					});
					break;

			case '2':
					LOG(INFO) << "Setting camera to 1080p30";
					guard( [](BMSDIBuffer *buffer ){
						bmAddVideoMode( buffer, CamNum,bmdModeHD1080p30 );
						bmAddReferenceSource( buffer, CamNum, BM_REF_SOURCE_PROGRAM );
						bmAddAutoExposureMode( buffer, CamNum, BM_AUTOEXPOSURE_SHUTTER );
					});
					break;

			case '3':
					LOG(INFO) << "Setting camera to 1080p60";
					guard( [](BMSDIBuffer *buffer ){
						bmAddVideoMode( buffer, CamNum,bmdModeHD1080p6000 );
						bmAddReferenceSource( buffer, CamNum, BM_REF_SOURCE_PROGRAM );
						bmAddAutoExposureMode( buffer, CamNum, BM_AUTOEXPOSURE_SHUTTER );
					});
					break;

			// case '3':
			// 		LOG(INFO) << "Sending 2160p25 reference to cameras";
			// 		guard( [](BMSDIBuffer *buffer ){				bmAddVideoMode( buffer, CamNum,bmdMode4K2160p25 );});
			// 		break;


			case '`':
				LOG(INFO) << "Updating camera";
					camState.updateCamera();
					break;

			case '\\':
			   if( recorder->isRecording() ) {
			           LOG(INFO) << "Stopping recording";
			           recorder->close();
			   } else {
			           LOG(INFO) << "Starting recording";

			           libblackmagic::ModeConfig config( decklink.input().currentConfig() );
								 ModeParams params( config.params() );

								 if( params.valid() ) {

								 const int numStreams = config.do3D() ? 2 : 1;
								 LOG(INFO) << "Opening video " << params.width << " x " << params.height << " with " << numStreams << " streams";
			          if( !recorder->open( params.width, params.height, params.frameRate, numStreams ) ) {
			                   LOG(WARNING) << "Unable to start recorder!";
			           }
							 } else {
								 LOG(WARNING) << "Bad configuration from the decklink";
							 }
			   }
				 break;


		case '9':
				LOG(INFO) << "Enabling overlay";
					guard( [](BMSDIBuffer *buffer ){ bmAddOverlayEnable( buffer, CamNum, 0x3 );});
				break;

		case '0':
				 LOG(INFO) << "Enabling overlay";
				 guard( [](BMSDIBuffer *buffer ){	bmAddOverlayEnable( buffer, CamNum, 0x0 );});
				 break;


		case 'q':
				keepGoing = false;
				break;
	}

}


int main( int argc, char** argv )
{
	libg3logger::G3Logger logger("bmRecorder");

	signal( SIGINT, signal_handler );

	CLI::App app{"Simple BlackMagic camera recorder"};

	int verbosity = 0;
	app.add_flag("-v", verbosity, "Additional output (use -vv for even more!)");

	bool do3D = false;
	app.add_flag("--do-3d",do3D, "Enable 3D modes");

	bool noDisplay = false;
	app.add_flag("--no-display,-x", noDisplay, "Disable display");

	string desiredModeString = "1080p2997";
	app.add_option("--mode,-m", desiredModeString, "Desired mode");

	bool doConfigCamera = false;
	app.add_flag("--config-camera,-c", doConfigCamera, "If enabled, send initialization info to the cameras");

	bool doListCards = false;
	app.add_flag("--list-cards", doListCards, "List Decklink cards in the system then exit");

	bool doListInputModes = false;
	app.add_flag("--list-input-modes", doListInputModes, "List Input modes then exit");

	int stopAfter = -1;
	app.add_option("--stop-after", stopAfter, "Stop after N frames");

	bool doSonar = false;
	app.add_flag("-s,--sonar", doSonar, "Record Oculus sonar");

  string sonarIp("auto");
  app.add_option("--sonar-ip", sonarIp, "IP address of sonar or \"auto\" to automatically detect.");

	string outputDir;
	app.add_option("--output,-o", outputDir, "Output dir");

	float previewScale = 0.5;
	app.add_option("--preview-scale", previewScale, "Scale of preview window");

	CLI11_PARSE(app, argc, argv);

	switch(verbosity) {
		case 1:
			logger.stderrHandle->call( &ColorStderrSink::setThreshold, INFO );
			break;
		case 2:
			logger.stderrHandle->call( &ColorStderrSink::setThreshold, DEBUG );
			break;
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

	// Handle the one-off commands
	if( doListCards || doListInputModes ) {
			if(doListCards) deckLink.listCards();
			if(doListInputModes) deckLink.listInputModes();
		return 0;
	}

	BMDDisplayMode mode = stringToDisplayMode( desiredModeString );
	if( (mode == bmdModeUnknown) || ( mode == bmdModeDetect) ) {
		LOG(WARNING) << "Card will always attempt automatic detection, starting in HD1080p2997 mode";
		mode = bmdModeHD1080p2997;
	} else {
		LOG(WARNING) << "Starting in mode " << desiredModeString;
	}

	//  Input should always auto-detect
	deckLink.input().enable( mode, true, do3D );
	deckLink.output().enable( mode );

	recorder.reset( new VideoRecorder( outputDir ) );

	if( doSonar ) sonar.reset( new SonarClient( sonarIp, recorder ));

	int count = 0, miss = 0, displayed = 0;

	CameraState cameraState( deckLink.output().sdiProtocolBuffer() );

	LOG(DEBUG) << "Starting streams";
	if( !deckLink.startStreams() ) {
			LOG(WARNING) << "Unable to start streams";
			exit(-1);
	}

	libblackmagic::InputHandler::MatPair rawImages, scaledImages;

	while( keepGoing ) {

		++count;
		if((stopAfter > 0) && (count > stopAfter)) { break; }

		if( !deckLink.input().queue().wait_for_pop( rawImages, std::chrono::milliseconds(100) ) ) {
			// No input

			// check for keyboard input
			continue;
		}


		unsigned int numImages = (rawImages[1].empty() ? 1 : 2);

		if( recorder->isRecording() ) {
			for( unsigned int i=0; i < numImages; ++i ) {
					recorder->addMat( rawImages[i], i );
			}
			recorder->advanceFrame();
		}


		if( noDisplay == false )  {

			for( unsigned int i=0; i < numImages; ++i ) {
				cv::Mat tmp;
				cv::resize( rawImages[i], tmp, cv::Size(), previewScale, previewScale  );

				scaledImages[i] = cv::Mat(tmp.rows, tmp.cols, CV_8UC3 );

				// Image from camera is BGRA  map to RGB
				int from_to[] = { 0,0, 1,1, 2,2 };
				cv::mixChannels( &tmp, 1, &(scaledImages[i]), 1, from_to, 3 );
				//cv::cvtColor( tmp, scaledImages[i], cv::COLOR_BGRA2RGB );
				//cv::extractChannel(tmp, scaledImages[i], 0 );
			}

			// Display images

			if( numImages == 1 ) {
				cv::imshow("Image", scaledImages[0]);
			} else if ( numImages == 2 ) {

				cv::Mat composite( cv::Size( scaledImages[0].size().width + scaledImages[0].size().width,
				std::max(scaledImages[0].size().height, scaledImages[1].size().height )), scaledImages[0].type() );

				if( !scaledImages[0].empty() ) {
					cv::Mat leftROI( composite, cv::Rect(0,0,scaledImages[0].size().width,scaledImages[0].size().height) );
					scaledImages[0].copyTo( leftROI );
				}

				if( !scaledImages[1].empty() ) {
					cv::Mat rightROI( composite, cv::Rect(scaledImages[0].size().width, 0, scaledImages[1].size().width, scaledImages[1].size().height) );
					scaledImages[1].copyTo( rightROI );
				}

				cv::imshow("Composite", composite );
			}

		}

		char c = cv::waitKey(1);
		processKbInput( c, deckLink, cameraState );

		LOG_IF(INFO, (displayed % 50) == 0) << "Frame #" << displayed;
		++displayed;

	}

	 //std::chrono::duration<float> dur( std::chrono::steady_clock::now()  - start );

	recorder->close();

	LOG(INFO) << "End of main loop, stopping streams...";

	deckLink.stopStreams();
	if( sonar ) sonar->stop();


	// LOG(INFO) << "Recorded " << count << " frames in " <<   dur.count();
	// LOG(INFO) << " Average of " << (float)count / dur.count() << " FPS";
	// LOG(INFO) << "   " << miss << " / " << (miss+count) << " misses";
	// LOG_IF( INFO, displayed > 0 ) << "   Displayed " << displayed << " frames";



		return 0;
	}
