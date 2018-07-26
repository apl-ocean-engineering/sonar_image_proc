
#include <string>

#include <DeckLinkAPIVersion.h>

#include "libg3logger/g3logger.h"

#include "DeckLink.h"

namespace libblackmagic {

  using std::string;

  DeckLink::DeckLink( int cardNo )
  : _cardNo(cardNo),
    _do3D(false),
    _deckLink( nullptr ),
    _deckLinkInput( nullptr ),
    _deckLinkOutput( nullptr ),
    _inputHandler( nullptr  ),
    _outputHandler(nullptr)
  {
    //initialize();
  }

  DeckLink::~DeckLink()
  {
    if( _deckLinkOutput ) {
      // Disable the video input interface
      _deckLinkOutput->DisableVideoOutput();
      _deckLinkOutput->Release();
    }

    if( _deckLinkInput ) {
      _deckLinkInput->StopStreams();
      _deckLinkInput->Release();
    }

  }

  //==== Lazy constructors ====
  IDeckLink *DeckLink::deckLink()
  {
    if( !_deckLink ) createDeckLink();
    CHECK( (bool)_deckLink ) << "Error creating Decklink card";
    return _deckLink;
  }

  IDeckLinkInput *DeckLink::deckLinkInput()
  {
    if( !_deckLinkInput && !createVideoInput() ) {
      LOG(FATAL) << "Error creating video input";
      return NULL;
    }
    CHECK( (bool)_deckLinkInput ) << "_deckLinkInput not set";
    return _deckLinkInput;
  }

  IDeckLinkOutput *DeckLink::deckLinkOutput()
  {
    if( !_deckLinkOutput && !createVideoOutput() ) {
      LOG(FATAL) << "Error creating video output";
      return NULL;
    }
    CHECK( (bool)_deckLinkOutput ) << "_deckLinkOutput not set";
    return _deckLinkOutput;
  }

  InputHandler &DeckLink::inputHandler()
  {
    // InputHandler is created in parallel with deckLinkInput
    if( !_inputHandler ) { deckLinkInput(); }
    CHECK( (bool)_inputHandler ) << "_inputHandler not set";

    return *_inputHandler;
  }

  OutputHandler &DeckLink::outputHandler()
  {
    // OutputHandler is create in parallel with deckLinkOutput
    if( !_outputHandler ) { deckLinkOutput(); }
    CHECK( (bool)_outputHandler ) << "_outputHandler not set";

    return *_outputHandler;
  }

  //=================================================================
  // Configuration functions
  bool DeckLink::createDeckLink()
  {
    LOG(INFO) << "Using Decklink API  " << BLACKMAGIC_DECKLINK_API_VERSION_STRING;

    if( _deckLink ) {
      _deckLink->Release();
      _deckLink = NULL;
    }

    HRESULT result;

    IDeckLink *dl = nullptr;
    IDeckLinkIterator *deckLinkIterator = CreateDeckLinkIteratorInstance();

    // Index cards by number for now
    for( int i = 0; i <= cardNo(); i++ ) {
      if( (result = deckLinkIterator->Next(&dl)) != S_OK) {
        LOG(WARNING) << "Couldn't get information on DeckLink card " << i;
        return false;
      }
    }

    _deckLink = dl;
    CHECK( deckLink() != nullptr );

    free( deckLinkIterator );

    char *modelName, *displayName;
    if( deckLink()->GetModelName( (const char **)&modelName ) != S_OK ) {
      LOG(WARNING) << "Unable to query model name.";
    }

    if( deckLink()->GetDisplayName( (const char **)&displayName ) != S_OK ) {
      LOG(WARNING) << "Unable to query display name.";
    }

    LOG(INFO) << "Using card " << cardNo() << " model name: " << modelName << "; display name: " << displayName;

    free(modelName);
    free(displayName);

    return true;
  }

  bool DeckLink::createVideoInput( const BMDDisplayMode desiredMode )
  {
    HRESULT result;

    // Hardcode some parameters for now
    BMDVideoInputFlags inputFlags = bmdVideoInputFlagDefault;
    BMDPixelFormat pixelFormat = bmdFormat10BitYUV;
    //BMDTimecodeFormat m_timecodeFormat;

    // Get the input (capture) interface of the DeckLink device
    result = deckLink()->QueryInterface(IID_IDeckLinkInput, (void**)&_deckLinkInput);
    if (result != S_OK) {
      LOG(WARNING) << "Couldn't get input for Decklink";
      return false;
    }

    IDeckLinkAttributes* deckLinkAttributes = NULL;
    bool formatDetectionSupported;

    // Check the card supports format detection
    result = deckLink()->QueryInterface(IID_IDeckLinkAttributes, (void**)&deckLinkAttributes);
    if (result == S_OK)
    {

      // Check for various desired features
      result = deckLinkAttributes->GetFlag(BMDDeckLinkSupportsInputFormatDetection, &formatDetectionSupported);
      if (result != S_OK || !formatDetectionSupported)
      {
        LOG(WARNING) << "Format detection is not supported on this device";
        return false;
      } else {
        LOG(INFO) << "Enabling automatic format detection on input card.";
        inputFlags |= bmdVideoInputEnableFormatDetection;
      }

    }


    // Format detection still needs a valid mode to start with
    //idx = 0;
    //         }
    //         else
    //         {
    //                 idx = g_config.mdisplayModeIndex;
    //         }


    IDeckLinkDisplayModeIterator* displayModeIterator = NULL;
    IDeckLinkDisplayMode *displayMode = NULL;

    result = deckLinkInput()->GetDisplayModeIterator(&displayModeIterator);
    if (result != S_OK) {
      LOG(WARNING) << "Unable to get DisplayModeIterator";
      return false;
    }

    // Iterate through available modes
    while( displayModeIterator->Next( &displayMode ) == S_OK ) {

      // char *displayModeName = nullptr;
      // if( displayMode->GetName( (const char **)&displayModeName) != S_OK ) {
      //   LOG(WARNING) << "Unable to get name of DisplayMode";
      //   return false;
      // }

      // BMDTimeValue timeValue = 0;
      // BMDTimeScale timeScale = 0;
      //
      // if( displayModeItr->GetFrameRate( &timeValue, &timeScale ) != S_OK ) {
      //   LOG(WARNING) << "Unable to get DisplayMode frame rate";
      //   return;
      // }
      //
      // float frameRate = (timeScale != 0) ? float(timeValue)/timeScale : float(timeValue);
      //
      // LOG(INFO) << "Card supports display mode \"" << displayModeName << "\"    " <<
      // displayModeItr->GetWidth() << " x " << displayModeItr->GetHeight() <<
      // ", " << 1.0/frameRate << " FPS";


      if( displayMode->GetDisplayMode() == desiredMode ) {

        //Check flags
        BMDDisplayModeFlags flags = displayMode->GetFlags();

        if( do3D() ) {
          if( !(flags & bmdDisplayModeSupports3D ) )
          {
            LOG(WARNING) << "3D Support requested but not available in this display mode";
          } else {
            LOG(INFO) << "Enabling 3D support detection on input card.";
            inputFlags |= bmdVideoInputDualStream3D;
          }

        }

        // Check display mode is supported with given options
        BMDDisplayModeSupport displayModeSupported;
        result = deckLinkInput()->DoesSupportVideoMode(displayMode->GetDisplayMode(),
        pixelFormat,
        inputFlags,
        &displayModeSupported, NULL);

        if (result != S_OK) {
          LOG(WARNING) << "Error checking if DeckLinkInput supports this mode";
          return false;
        }

        if (displayModeSupported == bmdDisplayModeNotSupported)
        {
          LOG(WARNING) <<  "The display mode is not supported with the selected pixel format on this input";
          return false;
        }

        // If you've made it here, great!
        break;
      }

      // free( displayModeName );
    }

    free( displayModeIterator );

    if( ! deckLinkInput() ) {
      // Failed to find a good mode
      LOG(FATAL) << "Unable to find a video input mode with the desired properties";
      goto bail;
    }

    // Configure the capture callback ... needs an output to create frames for conversion
    CHECK( deckLinkOutput() != nullptr );
    _inputHandler = new InputHandler( deckLinkInput(), deckLinkOutput(),  displayMode );

    // Made it this far?  Great!
    result = deckLinkInput()->EnableVideoInput(displayMode->GetDisplayMode(),
                                              pixelFormat,
                                              inputFlags);
                                              if (result != S_OK)
    {
      LOG(WARNING) << "Failed to enable video input. Is another application using the card?";
      goto bail;
    }

    return true;

    bail:
    free( displayMode );

    return false;
  }


  bool DeckLink::createVideoOutput( const BMDDisplayMode desiredMode )
  {
    // Video mode parameters
    //    const BMDDisplayMode      kDisplayMode = bmdModeHD1080i50;
    BMDVideoOutputFlags outputFlags  = bmdVideoOutputVANC;
    IDeckLinkDisplayMode *displayMode = nullptr;

    HRESULT result;

    // Obtain the output interface for the DeckLink device
    {
      IDeckLinkOutput *deckLinkOutput = nullptr;
      result = deckLink()->QueryInterface(IID_IDeckLinkOutput, (void**)&deckLinkOutput);
      if(result != S_OK)
      {
        LOGF(WARNING, "Could not obtain the IDeckLinkInput interface - result = %08x\n", result);
        return false;
      }

      _deckLinkOutput = deckLinkOutput;
    }
    CHECK( deckLinkOutput() != nullptr );

    if( do3D() ) {
      outputFlags |= bmdVideoOutputDualStream3D;
    }

    BMDDisplayModeSupport support;

    if( deckLinkOutput()->DoesSupportVideoMode( desiredMode, 0, outputFlags, &support, &displayMode ) != S_OK) {
      LOG(WARNING) << "Unable to find a supported output mode";
      return false;
    }

    if( support == bmdDisplayModeNotSupported ) {
      LOG(WARNING) << "Display mode not supported";
      return false;
    }


    // Enable video output
    result = deckLinkOutput()->EnableVideoOutput(desiredMode, outputFlags);
    if(result != S_OK)
    {
      LOGF(WARNING, "Could not enable video output - result = %08x\n", result);
      return false;
    }

    result = displayMode->GetFrameRate( &_outputTimeValue, &_outputTimeScale );
    if( result != S_OK ) {
      LOG(WARNING) << "Unable to get time rate information for output...";
      return false;
    }

    // Set the callback object to the DeckLink device's output interface
    _outputHandler = new OutputHandler( deckLinkOutput(), displayMode );
    result = deckLinkOutput()->SetScheduledFrameCompletionCallback( _outputHandler );
    if(result != S_OK)
    {
      LOGF(WARNING, "Could not set callback - result = %08x\n", result);
      return false;
    }

    displayMode->Release();


    return true;
  }


  //=================================================================

  bool DeckLink::startStreams( void )
  {
    //
    //  result = g_deckLinkInput->EnableAudioInput(bmdAudioSampleRate48kHz, g_config.m_audioSampleDepth, g_config.m_audioChannels);
    //  if (result != S_OK)
    //          goto bail;
    //

    if( deckLinkOutput() ) {
      LOG(INFO) << "Starting DeckLink output";

      HRESULT result = deckLinkOutput()->StartScheduledPlayback(0, _outputTimeScale, 1.0);
      if(result != S_OK)
      {
        LOG(WARNING) << "Could not start video output - result = " << std::hex << result;
      }

    }


    HRESULT result = deckLinkInput()->StartStreams();
    if (result != S_OK) {
      LOG(WARNING) << "Failed to start input streams";
      return false;
    }

    return true;
  }

  void DeckLink::stopStreams( void )
  {
    inputHandler().stopStreams();

    // LOG(INFO) << "Pausing DeckLinkInput streams";
    // if ( _deckLinkInput->PauseStreams() != S_OK) {
    //   LOG(WARNING) << "Failed to pause streams";
    //   return;
    // }
    //
    //
    // LOG(INFO) << "Flushing DeckLinkInput streams";
    // if ( _deckLinkInput->FlushStreams() != S_OK) {
    //   LOG(WARNING) << "Failed to flush streams";
    //   return;
    // }


    if( deckLinkOutput() ) {
      LOG(INFO) << "Stopping DeckLinkOutput streams";
      // // And stop after one frame
      BMDTimeValue actualStopTime;
      HRESULT result = deckLinkOutput()->StopScheduledPlayback(0, &actualStopTime, _outputTimeScale);
      if(result != S_OK)
      {
        LOG(WARNING) << "Could not stop video playback - result = " << std::hex << result;
      }

      LOG(INFO) << "     ...done";
    }
  }

  bool DeckLink::grab( void )
  {
    // TODO.  Go back and check how many copies are being made...
    _grabbedImage = cv::Mat();

    // while( inputHandler().queue().try_and_pop(_grabbedImage) ) { LOG(INFO) << "Pop!  Queue now " << inputHandler().queue().size(); }
    //
    // if( !_grabbedImage.empty() ) return true;

    // If there was nothing in the queue, wait
    if( inputHandler().queue().wait_for_pop(_grabbedImage, std::chrono::milliseconds(100) ) == false ) {
      LOG(WARNING) << "Timeout waiting for image in image queue";
      return false;
    }

    if( !_grabbedImage.empty() ) return true;

    return false;
  }

  int DeckLink::getRawImage( int i, cv::Mat &mat )
  {
    switch(i) {
      case 0:
              mat = _grabbedImage;
              return 1;
              break;

      default:
              return 0;
    }

    return 0;
  }

  // ImageSize DeckLink::imageSize( void ) const
  // {
  //   return _inputHandler->imageSize();
  // }

}
