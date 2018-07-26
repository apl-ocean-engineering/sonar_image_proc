
#include <iostream>
#include <thread>

#include "libg3logger/g3logger.h"

#include "libblackmagic/InputHandler.h"

namespace libblackmagic {

  const int maxDequeDepth = 10;

  InputHandler::InputHandler( IDeckLinkInput *input,
    IDeckLinkOutput *output,
    IDeckLinkDisplayMode *mode )
    : _frameCount(0),
    _deckLinkInput(input),
    _deckLinkOutput(output),
    _mode(mode)
    {
      _deckLinkInput->SetCallback(this);
    }

    ULONG InputHandler::AddRef(void) { return 1; }

    ULONG InputHandler::Release(void) { return 1; }


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
                  << ((formatFlags & bmdDetectedVideoInputDualStream3D) ? " with 3D" : "");

        if (displayModeName) free(displayModeName);

        if(_deckLinkInput) {
          _deckLinkInput->StopStreams();

          BMDVideoInputFlags m_inputFlags = bmdVideoInputFlagDefault | bmdVideoInputEnableFormatDetection;

          if( formatFlags & bmdDetectedVideoInputDualStream3D ) {
            LOG(INFO) << "Enabled 3D at new input format";
            m_inputFlags != bmdVideoInputDualStream3D;
          }

          result = _deckLinkInput->EnableVideoInput(mode->GetDisplayMode(), pixelFormat, m_inputFlags);
          if (result != S_OK) {
            LOG(WARNING) << "Failed to switch video mode";
            return result;
          }

          _deckLinkInput->StartStreams();
        }

        _mode = mode;

        return S_OK;
      }


      void InputHandler::stopStreams() {

        LOG(INFO) << "(" << std::this_thread::get_id() << ") Stopping DeckLinkInput streams";
        if (_deckLinkInput->StopStreams() != S_OK) {
          LOG(WARNING) << "Failed to stop input streams";
        }
        LOG(INFO) << "    ...done";

        //_thread.doDone();
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
