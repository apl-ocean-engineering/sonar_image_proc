
#include "libg3logger/g3logger.h"

#include "serdprecorder/SerdpRecorder.h"


namespace serdprecorder {

  using namespace libblackmagic;
  using std::string;


  SerdpRecorder::SerdpRecorder( void )
    : _keepGoing( true ),
      _deckLink( new DeckLink() ),
      _camState( new CameraState( _deckLink->output().sdiProtocolBuffer() ) ),
      _recorder( new VideoRecorder() ),
      _sonar( nullptr ),
      _display( new OpenCVDisplay( *this ) )
  {}



  int SerdpRecorder::run( int argc, char **argv )
  {
    libg3logger::G3Logger logger("bmRecorder");

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
        if(doListCards) _deckLink->listCards();
        if(doListInputModes) _deckLink->listInputModes();
      return 0;
    }

    BMDDisplayMode mode = stringToDisplayMode( desiredModeString );
    if( (mode == bmdModeUnknown) || ( mode == bmdModeDetect) ) {
      LOG(WARNING) << "Card will always attempt automatic detection, starting in HD1080p2997 mode";
      mode = bmdModeHD1080p2997;
    } else {
      LOG(WARNING) << "Starting in mode " << desiredModeString;
    }

    _recorder->setOutputDir( outputDir );
    //_recorder->setDoSonar( doSonar );

    _display->setEnabled( !noDisplay );

    //  Input should always auto-detect
    _deckLink->input().enable( mode, true, do3D );
    _deckLink->output().enable( mode );

    if( doSonar ) {
      LOG(INFO) << "Enabling sonar";
      _sonar.reset( new SonarClient( sonarIp, _recorder, _display ) );
      _sonar->start();
    }

    int count = 0, miss = 0, displayed = 0;

    LOG(DEBUG) << "Starting streams";
    if( !_deckLink->startStreams() ) {
        LOG(WARNING) << "Unable to start streams";
        exit(-1);
    }

    libblackmagic::InputHandler::MatVector rawImages, scaledImages;
    std::chrono::system_clock::time_point prevImage = std::chrono::system_clock::now();

    while( _keepGoing ) {

      ++count;
      if((stopAfter > 0) && (count > stopAfter)) { break; }

      if( !_deckLink->input().queue().wait_for_pop( rawImages, std::chrono::milliseconds(100) ) ) {
        // No input

        // check for keyboard input
        continue;
      }

      // Compute dt
      std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

      auto dt = now-prevImage;

//std::chrono::milliseconds(dt).count()
      LOG(WARNING) << "dt = " << float(dt.count())/1e6 << " ms";

      prevImage = now;

      unsigned int numImages = rawImages.size();

      // Process each step in its own thread
      if( _recorder->isRecording() ) _recorder->addMats( rawImages );

      // Reap all threads
      _display->showVideo( rawImages );

      LOG_IF(INFO, (displayed % 50) == 0) << "Frame #" << displayed;
      ++displayed;

    }

     //std::chrono::duration<float> dur( std::chrono::steady_clock::now()  - start );

    if( _recorder->isRecording() ) _recorder->close();

    LOG(INFO) << "End of main loop, stopping streams...";

    _deckLink->stopStreams();
    if( _sonar ) _sonar->stop();


    // LOG(INFO) << "Recorded " << count << " frames in " <<   dur.count();
    // LOG(INFO) << " Average of " << (float)count / dur.count() << " FPS";
    // LOG(INFO) << "   " << miss << " / " << (miss+count) << " misses";
    // LOG_IF( INFO, displayed > 0 ) << "   Displayed " << displayed << " frames";



      return 0;

  }

  void SerdpRecorder::handleKey( const char c ) {

  	std::shared_ptr<SharedBMSDIBuffer> sdiBuffer( _deckLink->output().sdiProtocolBuffer() );
    const int CamNum = 1;

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
  						auto val = _camState->apertureInc();
  	 					// LOG(INFO) << "Sending aperture increment " << val.ord << " , " << val.val << " , " << val.str;
  						LOG(INFO) << "Set aperture to " << val;
  					}
   					// guard( []( BMSDIBuffer *buffer ){	bmAddOrdinalApertureOffset( buffer, CamNum, 1 ); });
   					break;
   			case ';':
  					{
  						// Send negative aperture decrement
  						auto val = _camState->apertureDec();
  						//LOG(INFO) << "Sending aperture decrement " << val.ord << " , " << val.val << " , " << val.str;
  						LOG(INFO) << "Set aperture to " << val;
  					// guard( []( BMSDIBuffer *buffer ){	bmAddOrdinalApertureOffset( buffer, CamNum, -1 ); });
  					}
   					break;

  			//=== Shutter increment/decrement ===
  			case '.':
   					LOG(INFO) << "Sending shutter increment to camera";
  					_camState->exposureInc();
   					break;
   			case '/':
   					LOG(INFO) << "Sending shutter decrement to camera";
  					_camState->exposureDec();
   					break;

  			//=== Gain increment/decrement ===
  			case 'z':
   					LOG(INFO) << "Sending gain increment to camera";
  					_camState->gainInc();
   					break;
   			case 'x':
   					LOG(INFO) << "Sending gain decrement to camera";
  					_camState->gainDec();
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
  					_camState->updateCamera();
  					break;

  			case '\\':
  			   if( _recorder->isRecording() ) {
  			           LOG(INFO) << "Stopping recording";
  			           _recorder->close();
  			   } else {
	           LOG(INFO) << "Starting recording";

	           const libblackmagic::ModeConfig config( _deckLink->input().currentConfig() );
						 const ModeParams params( config.params() );

						 if( params.valid() ) {
  						 const int numStreams = config.do3D() ? 2 : 1;
  						 LOG(INFO) << "Opening video " << params.width << " x " << params.height << " with " << numStreams << " streams";
  	           if( !_recorder->open( params.width, params.height, params.frameRate, numStreams ) ) {
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
  				_keepGoing = false;
  				break;
  	}

  }


}
