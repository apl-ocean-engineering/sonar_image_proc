
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
  _deckLinkOutput( nullptr )
  {
    _deckLinkInput->SetCallback(this);
  }

  ULONG InputHandler::AddRef(void) { return 1; }

  ULONG InputHandler::Release(void) { return 1; }

  // Create based on current configuration
  IDeckLinkInput *InputHandler::deckLinkInput()
  {
    if( _deckLinkInput ) return _deckLinkInput;

    HRESULT result;

    // Get the input (capture) interface of the DeckLink device
    result = _deckLink->QueryInterface(IID_IDeckLinkInput, (void**)&_deckLinkInput);
    //if (result != S_OK) {
      //LOG(FATAL) << "Couldn't get input for Decklink";
    //}

    CHECK( _deckLinkInput != NULL ) << "Couldn't get input for Decklink";
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
      _config.set3D( displayMode->GetFlags() & bmdDetectedVideoInputDualStream3D );

      _enabled = true;
      return true;
    }

        // return true;
        //
        // bail:
        // free( displayMode );

        /// Don't need to search this way?  Only advantage?  Can check if 3D is
        // available for mode before calling DoesSupportVideoMode
        //   result = deckLinkInput()->GetDisplayModeIterator(&displayModeIterator);
        //   if (result != S_OK) {
        //     LOG(WARNING) << "Unable to get DisplayModeIterator";
        //     return false;
        //   }
        //
        //   // Find the targetMode
        //   while( displayModeIterator->Next( &displayMode ) == S_OK ) {
        //     if( displayMode->GetDisplayMode() == targetMode ) {
        //
        //       //Check flags
        //       BMDDisplayModeFlags flags = displayMode->GetFlags();
        //
        //       if( config()->do3D() ) {
        //         if( !(flags & bmdDisplayModeSupports3D ) )
        //         {
        //           LOG(WARNING) << "!! 3D Support requested but not available in this display mode";
        //         } else {
        //           inputFlags |= bmdVideoInputDualStream3D;
        //           LOG(INFO) << "* Enabling 3D support detection on input card.  " << inputFlags;
        //         }
        //       }
        //
        //       // Check display mode is supported with given options
        //       BMDDisplayModeSupport displayModeSupported;
        //       result = _deckLinkInput->DoesSupportVideoMode(displayMode->GetDisplayMode(),
        //                                                       pixelFormat,
        //                                                       inputFlags,
        //                                                       &displayModeSupported, &displayMode);
        //
        //       if (result != S_OK) {
        //         LOG(WARNING) << "Error checking if DeckLinkInput supports this mode";
        //         return false;
        //       }
        //
        //       if (displayModeSupported == bmdDisplayModeNotSupported)
        //       {
        //         LOG(WARNING) <<  "The display mode is not supported with the selected pixel format on this input";
        //         return false;
        //       }
        //
        //       // If you've made it here, great!
        //       break;
        //     }
        // }
        // displayModeIterator->Release();

      //   return false;
      // }


      IDeckLinkOutput *InputHandler::deckLinkOutput()
      {
        if( _deckLinkOutput ) return _deckLinkOutput;

        HRESULT result = _deckLink->QueryInterface(IID_IDeckLinkOutput, (void**)&_deckLinkOutput);
        if (result != S_OK) {
          LOG(WARNING) << "Couldn't get output for Decklink";
          return nullptr;
        }

        return _deckLinkOutput;
      }


      bool InputHandler::startStreams() {
        if( _enabled && !enable() ) return false;

        HRESULT result = _deckLinkInput->StartStreams();
        if (result != S_OK) {
          LOG(WARNING) << "Failed to start input streams";
          return false;
        }

        return true;
      }

      bool InputHandler::stopStreams() {

        LOG(INFO) << "(" << std::this_thread::get_id() << ") Stopping DeckLinkInput streams";
        if (_deckLinkInput->StopStreams() != S_OK) {
          LOG(WARNING) << "Failed to stop input streams";
        }
        LOG(INFO) << "    ...done";

        //_thread.doDone();

        return true;
      }




      // Callbacks are called in a private thread....
      HRESULT InputHandler::VideoInputFrameArrived(IDeckLinkVideoInputFrame* videoFrame,
        IDeckLinkAudioInputPacket* audioFrame)
        {
          IDeckLinkVideoFrame *rightEyeFrame = nullptr;
          IDeckLinkVideoFrame3DExtensions *threeDExtensions = nullptr;

          // Handle Video Frame
          if (videoFrame)
          {

            // If 3D mode is enabled we retreive the 3D extensions interface which gives.
            // us access to the right eye frame by calling GetFrameForRightEye() .
            if( videoFrame->QueryInterface(IID_IDeckLinkVideoFrame3DExtensions, (void **) &threeDExtensions) == S_OK ) {
              LOG(INFO) << "Checking 3D extensions";

              if(threeDExtensions->GetFrameForRightEye(&rightEyeFrame) != S_OK) {
                LOG(INFO) << "Error getting right eye frame";
              }
            }
            if (threeDExtensions) threeDExtensions->Release();



            if (videoFrame->GetFlags() & bmdFrameHasNoInputSource)
            {
              LOG(WARNING) << "(" << std::this_thread::get_id()
              << ") Frame received (" << _frameCount
              << ") - No input signal detected";
            }
            else
            {
              _frameCount++;

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

              if( rightEyeFrame ) {
                rightEyeFrame->AddRef();

                std::thread t = processInThread( rightEyeFrame, true );
                t.detach();

                //rightEyeFrame->Release();
              }

            }
          }


          return S_OK;
        }



        // Callback if bmdVideoInputEnableFormatDetection was set when
        // enabling video input
        HRESULT InputHandler::VideoInputFormatChanged(BMDVideoInputFormatChangedEvents events, IDeckLinkDisplayMode *mode,
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

            if(_deckLinkInput) {
              _deckLinkInput->StopStreams();

              BMDVideoInputFlags m_inputFlags = bmdVideoInputFlagDefault | bmdVideoInputEnableFormatDetection;

              if( formatFlags & bmdDetectedVideoInputDualStream3D ) {
                LOG(INFO) << "Enabled 3D at new input format";
                m_inputFlags |= bmdVideoInputDualStream3D;
              }

              result = _deckLinkInput->EnableVideoInput(mode->GetDisplayMode(), pixelFormat, m_inputFlags);
              if (result != S_OK) {
                LOG(WARNING) << "Failed to switch video mode";
                return result;
              }

              _deckLinkInput->StartStreams();
            }

            _config.setMode( mode->GetDisplayMode() );
            _config.set3D( formatFlags & bmdDetectedVideoInputDualStream3D );

            return S_OK;
          }
          bool InputHandler::process(  IDeckLinkVideoFrame *videoFrame, bool isRight )
          {

            LOG_IF(INFO, isRight) << "Processing right frame...";

            cv::Mat out;

            switch (videoFrame->GetPixelFormat()) {
              case bmdFormat8BitYUV:
              {
                void* data;
                if ( videoFrame->GetBytes(&data) != S_OK )
                return false;
                cv::Mat mat = cv::Mat(videoFrame->GetHeight(), videoFrame->GetWidth(), CV_8UC2, data,
                videoFrame->GetRowBytes());
                cv::cvtColor(mat, out, cv::COLOR_YUV2BGR ); //_UYVY);
                return true;
              }
              case bmdFormat8BitBGRA:
              {
                void* data;
                if ( videoFrame->GetBytes(&data) != S_OK )
                return false;

                cv::Mat mat = cv::Mat(videoFrame->GetHeight(), videoFrame->GetWidth(), CV_8UC4, data);
                cv::cvtColor(mat, out, cv::COLOR_BGRA2BGR);
                return true;
              }
              default:
              {
                //LOG(INFO) << "Converting through Blackmagic VideoConversionInstance";
                IDeckLinkMutableVideoFrame*     dstFrame = NULL;

                //CvMatDeckLinkVideoFrame cvMatWrapper(videoFrame->GetHeight(), videoFrame->GetWidth());
                HRESULT result = _deckLinkOutput->CreateVideoFrame( videoFrame->GetWidth(), videoFrame->GetHeight(),
                videoFrame->GetWidth() * 4, bmdFormat8BitBGRA, bmdFrameFlagDefault, &dstFrame);
                if (result != S_OK)
                {
                  LOG(WARNING) << "Failed to create destination video frame";
                  return false;
                }

                IDeckLinkVideoConversion *converter =  CreateVideoConversionInstance();

                //LOG(WARNING) << "Converting " << std::hex << videoFrame->GetPixelFormat() << " to " << dstFrame->GetPixelFormat();
                result =  converter->ConvertFrame(videoFrame, dstFrame);

                if (result != S_OK ) {
                  LOG(WARNING) << "Failed to do conversion " << std::hex << result;
                  return false;
                }

                void *buffer = nullptr;
                if( dstFrame->GetBytes( &buffer ) != S_OK ) {
                  LOG(WARNING) << "Unable to get bytes from dstFrame";
                  return false;
                }
                cv::Mat srcMat( cv::Size(dstFrame->GetWidth(), dstFrame->GetHeight()), CV_8UC4, buffer, dstFrame->GetRowBytes() );
                //cv::cvtColor(srcMat, out, cv::COLOR_BGRA2BGR);
                cv::resize( srcMat, out, cv::Size(), 0.25, 0.25  );

                dstFrame->Release();
                videoFrame->Release();
              }
            }


            if( queue().size() < maxDequeDepth && !out.empty() ) {
              queue().push( out );
              //LOG(INFO) << "Push!  Queue now " << queue().size();
              return true;
            }


            LOG(WARNING) << "Image queue full (" << queue().size() << "), unable to queue more images";
            return false;

            //   if (rightEyeFrame)
            //   {
            //     rightEyeFrame->GetBytes(&frameBytes);
            //     write(g_videoOutputFile, frameBytes, videoFrame->GetRowBytes() * videoFrame->GetHeight());
            //   }

            //LOG(INFO) << "Finishing process in thread " << std::this_thread::get_id();

          }





        }
