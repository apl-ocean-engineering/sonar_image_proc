
#include <iostream>
#include <thread>

#include "libg3logger/g3logger.h"

#include "libblackmagic/InputHandler.h"

namespace libblackmagic {

  const int maxDequeDepth = 10;

  InputHandler::InputHandler( IDeckLink *deckLink )
    : _frameCount(0),
    _config(),
    _enabled( false ),
    _deckLink(deckLink),
    _deckLinkInput(  nullptr ),
    _deckLinkOutput( nullptr ),
    _grabbedImages(),
    _queues()
  {
    deckLink->AddRef(); }

  InputHandler::~InputHandler() {
    if( _deckLinkInput ) _deckLinkInput->Release();
    if( _deckLinkOutput ) _deckLinkOutput->Release();
    if( _deckLink ) _deckLink->Release();
  }

  ULONG InputHandler::AddRef(void) {
    return 1;
  }

  ULONG InputHandler::Release(void) {
    return 1;
  }

  // Create based on current configuration
  IDeckLinkInput *InputHandler::deckLinkInput() {
    if( !_deckLinkInput ) {
      CHECK( _deckLink->QueryInterface(IID_IDeckLinkInput, (void**)&_deckLinkInput) == S_OK );
      CHECK( _deckLinkInput != NULL ) << "Couldn't get input for Decklink";
    }

    return _deckLinkInput;
  }

  bool InputHandler::enable( void ) {

    // Hardcode some parameters for now
    BMDVideoInputFlags inputFlags = bmdVideoInputFlagDefault;
    BMDPixelFormat pixelFormat = bmdFormat10BitYUV;

    IDeckLinkAttributes* deckLinkAttributes = NULL;
    bool formatDetectionSupported;

    // Check the card supports format detection
    auto result = _deckLink->QueryInterface(IID_IDeckLinkAttributes, (void**)&deckLinkAttributes);
    if (result != S_OK) {
      LOG(WARNING) << "Unable to query deckLinkAttributes";
      return false;
    }

    BMDDisplayMode targetMode = _config.mode();

    if( targetMode == bmdModeDetect ) {
      LOG(INFO) << "Automatic mode detection requested, checking for support";

      // Check for various desired features
      result = deckLinkAttributes->GetFlag(BMDDeckLinkSupportsInputFormatDetection, &formatDetectionSupported);
      if (result != S_OK || !formatDetectionSupported)
      {
        LOG(WARNING) << "* Format detection is not supported on this device";
        return false;
      } else {
        LOG(INFO) << "* Enabling automatic format detection on input card.";
        inputFlags |= bmdVideoInputEnableFormatDetection;
        targetMode = bmdModeHD1080p2997;
      }

    }

    if( _config.do3D() ) {
      inputFlags |= bmdVideoInputDualStream3D;
    }

    //  IDeckLinkDisplayModeIterator* displayModeIterator = NULL;
    IDeckLinkDisplayMode *displayMode = nullptr;

    // WHy iterate?  Just ask!
    BMDDisplayModeSupport displayModeSupported;
    result = deckLinkInput()->DoesSupportVideoMode(targetMode,
                                                    pixelFormat,
                                                    inputFlags,
                                                    &displayModeSupported,
                                                    &displayMode);


      if (result != S_OK) {
        LOG(WARNING) << "Error while checking if DeckLinkInput supports mode";
        return false;
      }

      if (displayModeSupported == bmdDisplayModeNotSupported) {
        LOG(WARNING) <<  "The display mode is not supported with the selected pixel format on this input";
        return false;
      }

      CHECK( displayMode != nullptr ) << "Unable to find a video input mode with the desired properties";

      deckLinkInput()->SetCallback(this);

      deckLinkInput()->DisableAudioInput();

      // Made it this far?  Great!
      if( S_OK != deckLinkInput()->EnableVideoInput(displayMode->GetDisplayMode(),
                                                    pixelFormat,
                                                    inputFlags) ) {
          LOG(WARNING) << "Failed to enable video input. Is another application using the card?";
          return false;
      }

      // Feed results

      LOG(INFO) << "DeckLinkInput complete!";

      // Update config with values
      _config.setMode( displayMode->GetDisplayMode() );
      //_config.set3D( displayMode->GetFlags() & bmdDetectedVideoInputDualStream3D );

      displayMode->Release();

      _enabled = true;
      return true;
    }

  IDeckLinkOutput *InputHandler::deckLinkOutput()
  {
    if( _deckLinkOutput ) return _deckLinkOutput;

    HRESULT result = _deckLink->QueryInterface(IID_IDeckLinkOutput, (void**)&_deckLinkOutput);
    if (result != S_OK) {
      CHECK( _deckLinkOutput != nullptr ) << "Couldn't get output for Decklink";
      return nullptr;
    }

    return _deckLinkOutput;
  }


  bool InputHandler::startStreams() {
    if( !_enabled && !enable() ) return false;

    LOG(INFO) << "Starting DeckLink inputs ....";

    HRESULT result = deckLinkInput()->StartStreams();
    if (result != S_OK) {
      LOG(WARNING) << "Failed to start input streams";
      return false;
    }

    LOG(INFO) << "     ... done";

    return true;
  }

bool InputHandler::stopStreams() {
  LOG(INFO) << " Stopping DeckLinkInput streams";
  if (deckLinkInput()->StopStreams() != S_OK) {
    LOG(WARNING) << "Failed to stop input streams";
  }
  LOG(INFO) << "    ...done";

  return true;
}



int InputHandler::grab( void ) {

  // TODO.  Go back and check how many copies are being made...
  _grabbedImages[0] = cv::Mat();
  int numImages = config().do3D() ? 2 : 1;

  for( auto i = 0; i < numImages; ++i ) {
    // If there was nothing in the queue, wait
    if( _queues[i].wait_for_pop(_grabbedImages[i], std::chrono::milliseconds(100) ) == false ) {
      LOG(WARNING) << "Timeout waiting for image queue " << i;
      return 0;
    }

  }

  // Formerly checked for empty Mats here.  Still do that?

  return numImages;
}

int InputHandler::getRawImage( int i, cv::Mat &mat ) {

  if( i == 0 || i == 1 ) {
    mat = _grabbedImages[i];
    return 1;
  }

 return 0;
}

// ImageSize InputHandler::imageSize( void ) const
// {
// //   return _inputHandler->imageSize();
// }




// Callbacks are called in a private thread....
HRESULT InputHandler::VideoInputFrameArrived(IDeckLinkVideoInputFrame* videoFrame, IDeckLinkAudioInputPacket* audioFrame)
{
  IDeckLinkVideoFrame *rightEyeFrame = nullptr;
  IDeckLinkVideoFrame3DExtensions *threeDExtensions = nullptr;

  // Drop audio first thing
  if( audioFrame ) audioFrame->Release();

  uint32_t availFrames;
  if( deckLinkInput()->GetAvailableVideoFrameCount( &availFrames ) == S_OK ) {
    LOG(DEBUG) << "videoInputFrameArrives; " << availFrames << " still available";
  } else {
    LOG(DEBUG) << "videoInputFrameArrives";
  }

  // Handle Video Frame
  if (videoFrame) {

    if (videoFrame->GetFlags() & bmdFrameHasNoInputSource)
    {
      LOG(WARNING) << "(" << std::this_thread::get_id()
                          << ") Frame received (" << _frameCount
                          << ") - No input signal detected";
    }
    else
    {
      // const char *timecodeString = nullptr;
      // if (g_config.m_timecodeFormat != 0)
      // {
      //   IDeckLinkTimecode *timecode;
      //   if (videoFrame->GetTimecode(g_config.m_timecodeFormat, &timecode) == S_OK)
      //   {
      //     timecode->GetString(&timecodeString);
      //   }
      // }

      LOG(DEBUG) << "(" << std::this_thread::get_id()
                  << ") Frame received (" << _frameCount
                  << ") " << videoFrame->GetRowBytes() * videoFrame->GetHeight()
                  << " bytes, " << videoFrame->GetWidth()
                  << " x " << videoFrame->GetHeight();

      // The AddRef will ensure the frame is valid after the end of the callback.
      videoFrame->AddRef();
      std::thread t = processInThread( videoFrame );
      t.detach();

      // If 3D mode is enabled we retreive the 3D extensions interface which gives.
      // us access to the right eye frame by calling GetFrameForRightEye() .
      if( videoFrame->QueryInterface(IID_IDeckLinkVideoFrame3DExtensions, (void **) &threeDExtensions) == S_OK ) {
        if(threeDExtensions->GetFrameForRightEye(&rightEyeFrame) != S_OK) {
          LOG(INFO) << "Error getting right eye frame";
        }

        LOG(DEBUG) << "(" << std::this_thread::get_id()
                    << ") Right frame received (" << _frameCount
                    << ") " << rightEyeFrame->GetRowBytes() * rightEyeFrame->GetHeight()
                    << " bytes, " << rightEyeFrame->GetWidth()
                    << " x " << rightEyeFrame->GetHeight();

        // The AddRef will ensure the frame is valid after the end of the callback.
        //rightEyeFrame->AddRef();
        std::thread t = processInThread( rightEyeFrame, 1 );
        t.detach();
      }
      if (threeDExtensions) threeDExtensions->Release();

      _frameCount++;
    }
  }


  return S_OK;
}



// Callback if bmdVideoInputEnableFormatDetection was set when
// enabling video input
HRESULT InputHandler::VideoInputFormatChanged(BMDVideoInputFormatChangedEvents events,
                                                IDeckLinkDisplayMode *mode,
                                                BMDDetectedVideoInputFormatFlags formatFlags)
  {
    LOG(INFO) << "(" << std::this_thread::get_id() << ") Received Video Input Format Changed";

    HRESULT result;
    char*   displayModeName = nullptr;
    BMDPixelFormat  pixelFormat = bmdFormat10BitYUV;

    if (formatFlags & bmdDetectedVideoInputRGB444) pixelFormat = bmdFormat10BitRGB;

    mode->GetName((const char**)&displayModeName);
    LOG(INFO) << "Video format changed to " << displayModeName << " "
                        << ((formatFlags & bmdDetectedVideoInputRGB444) ? "RGB" : "YUV")
                        << ((formatFlags & bmdDetectedVideoInputDualStream3D) ? " with 3D" : " not 3D");

    if (displayModeName) free(displayModeName);

    deckLinkInput()->PauseStreams();

      BMDVideoInputFlags m_inputFlags = bmdVideoInputFlagDefault | bmdVideoInputEnableFormatDetection;

      if( formatFlags & bmdDetectedVideoInputDualStream3D ) {
      //if( config().do3D() ) {
        LOG(INFO) << "Enabled 3D at new input format";
        m_inputFlags |= bmdVideoInputDualStream3D;
      }

      result = deckLinkInput()->EnableVideoInput(mode->GetDisplayMode(), pixelFormat, m_inputFlags);
      if (result != S_OK) {
        LOG(WARNING) << "Failed to switch video mode";
        return result;
      }

      _queues[0].flush();
      _queues[1].flush();

      _config.setMode( mode->GetDisplayMode() );
      _config.set3D( formatFlags & bmdDetectedVideoInputDualStream3D );

      deckLinkInput()->FlushStreams();
      deckLinkInput()->StartStreams();

    return S_OK;
  }


  //
  //
  //
  bool InputHandler::process(  IDeckLinkVideoFrame *videoFrame, int input )
  {

    std::string frameName( config().do3D() ? ((input==1) ? "[RIGHT]" : "[LEFT]") : "" );

    LOG(DEBUG) << frameName << " Processing frame...";

    cv::Mat out;

    switch (videoFrame->GetPixelFormat()) {
      case bmdFormat8BitYUV:
      {
        void* data;
        if ( videoFrame->GetBytes(&data) != S_OK ) goto bail;
        cv::Mat mat = cv::Mat(videoFrame->GetHeight(), videoFrame->GetWidth(), CV_8UC2, data,
        videoFrame->GetRowBytes());
        cv::cvtColor(mat, out, cv::COLOR_YUV2BGR ); //_UYVY);
        break;
      }
      case bmdFormat8BitBGRA:
      {
        void* data;
        if ( videoFrame->GetBytes(&data) != S_OK ) goto bail;

        cv::Mat mat = cv::Mat(videoFrame->GetHeight(), videoFrame->GetWidth(), CV_8UC4, data);
        cv::cvtColor(mat, out, cv::COLOR_BGRA2BGR);
        break;
      }
      default:
      {
        IDeckLinkMutableVideoFrame*     dstFrame = NULL;

        //CvMatDeckLinkVideoFrame cvMatWrapper(videoFrame->GetHeight(), videoFrame->GetWidth());
        LOG(DEBUG) << frameName << "Converting through Blackmagic VideoConversionInstance to " << videoFrame->GetWidth() << " x " << videoFrame->GetHeight();
        HRESULT result = deckLinkOutput()->CreateVideoFrame( videoFrame->GetWidth(), videoFrame->GetHeight(),
                                                            videoFrame->GetWidth() * 4, bmdFormat8BitBGRA, bmdFrameFlagDefault, &dstFrame);
        if (result != S_OK)
        {
          LOG(WARNING) << frameName << " Failed to create destination video frame";
          goto bail;
        }

        IDeckLinkVideoConversion *converter =  CreateVideoConversionInstance();

        //LOG(WARNING) << "Converting " << std::hex << videoFrame->GetPixelFormat() << " to " << dstFrame->GetPixelFormat();
        result =  converter->ConvertFrame(videoFrame, dstFrame);

        if (result != S_OK ) {
          LOG(WARNING) << frameName << " Failed to do conversion " << std::hex << result;
          goto bail;
        }

        void *buffer = nullptr;
        if( dstFrame->GetBytes( &buffer ) != S_OK ) {
          LOG(WARNING) << frameName << " Unable to get bytes from dstFrame";
          goto bail;
        }
        cv::Mat srcMat( cv::Size(dstFrame->GetWidth(), dstFrame->GetHeight()), CV_8UC4, buffer, dstFrame->GetRowBytes() );
        //cv::cvtColor(srcMat, out, cv::COLOR_BGRA2BGR);
        cv::resize( srcMat, out, cv::Size(), 0.25, 0.25  );

        dstFrame->Release();
      }
    }

    LOG(DEBUG) << frameName << " Release; " << videoFrame->Release() << " references remain";

    if( out.empty() ) {
      LOG(WARNING) << frameName << " Frame is empty?";
      goto bail;
    }

    while( _queues[input].size() >= maxDequeDepth && _queues[input].pop_and_drop()  ) {;}
    _queues[input].push( out );

    LOG(DEBUG) << frameName << " Push!" << " queue now " << _queues[input].size();
    return true;


bail:
    videoFrame->Release();
    return false;

  }






}
