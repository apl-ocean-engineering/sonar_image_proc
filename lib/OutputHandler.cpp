
#include <DeckLinkAPI.h>

#include <g3log/g3log.hpp>

#include "SDICameraControl.h"
#include "OutputHandler.h"

namespace libblackmagic {


	OutputHandler::OutputHandler( IDeckLink *deckLink )
			:  _deckLink(deckLink),
				_deckLinkOutput( nullptr ),
				_mode( nullptr ),
				_totalFramesScheduled(0),
				_buffer( new SharedBMSDIBuffer() ),
				_blankFrame( makeBlueFrame(deckLinkOutput, true ))
		{
			// _mode->AddRef();
			//
			// _mode->GetFrameRate( &_timeValue, &_timeScale );

			// Get timing information from mode

			// Pre-roll a few blank frames
			const int prerollFrames = 3;
			for( int i = 0; i < prerollFrames ; ++i ) {
				scheduleFrame(_blankFrame);
			}

		}

	OutputHandler::~OutputHandler(void)
	{
		if( _deckLinkOutput ) _deckLinkOutput->Release();
	}


	  IDeckLinkOutput *OutputHandler::deckLinkOutput()
	  {
			// 	if( _deckLinkOutput ) { return _deckLinkOutput; }
			//
	    // // Video mode parameters
	    // //    const BMDDisplayMode      kDisplayMode = bmdModeHD1080i50;
	    // BMDVideoOutputFlags outputFlags  = bmdVideoOutputVANC;
	    // IDeckLinkDisplayMode *displayMode = nullptr;
			//
	    // HRESULT result;
			//
	    // // Obtain the output interface for the DeckLink device
	    // {
	    //   IDeckLinkOutput *deckLinkOutput = nullptr;
	    //   result = deckLink()->QueryInterface(IID_IDeckLinkOutput, (void**)&deckLinkOutput);
	    //   if(result != S_OK)
	    //   {
	    //     LOGF(WARNING, "Could not obtain the IDeckLinkInput interface - result = %08x\n", result);
	    //     return false;
	    //   }
			//
	    //   _deckLinkOutput = deckLinkOutput;
	    // }
	    // CHECK( deckLinkOutput() != nullptr );
			//
	    // // Don't use 3D for output
	    // // if( do3D() ) {
	    // //   outputFlags |= bmdVideoOutputDualStream3D;
	    // // }
			//
	    // BMDDisplayModeSupport support;
			//
	    // if( deckLinkOutput()->DoesSupportVideoMode( desiredMode, 0, outputFlags, &support, &displayMode ) != S_OK) {
	    //   LOG(WARNING) << "Unable to find a supported output mode";
	    //   return false;
	    // }
			//
	    // if( support == bmdDisplayModeNotSupported ) {
	    //   LOG(WARNING) << "Display mode not supported";
	    //   return false;
	    // }
			//
			//
	    // // Enable video output
	    // result = deckLinkOutput()->EnableVideoOutput(desiredMode, outputFlags);
	    // if(result != S_OK)
	    // {
	    //   LOGF(WARNING, "Could not enable video output - result = %08x\n", result);
	    //   return false;
	    // }
			//
	    // result = displayMode->GetFrameRate( &_outputTimeValue, &_outputTimeScale );
	    // if( result != S_OK ) {
	    //   LOG(WARNING) << "Unable to get time rate information for output...";
	    //   return false;
	    // }
			//
	    // // Set the callback object to the DeckLink device's output interface
	    // _outputHandler = new OutputHandler( deckLinkOutput(), displayMode );
	    // result = deckLinkOutput()->SetScheduledFrameCompletionCallback( _outputHandler );
	    // if(result != S_OK)
	    // {
	    //   LOGF(WARNING, "Could not set callback - result = %08x\n", result);
	    //   return false;
	    // }
			//
	    // displayMode->Release();
			//
	    // LOG(INFO) << "DeckLinkOutput complete!";
	    // return true;
	  }

	void OutputHandler::scheduleFrame( IDeckLinkVideoFrame *frame, uint8_t count )
	{
		//LOG(INFO) << "Scheduled frame " << _totalFramesScheduled;
		_deckLinkOutput->ScheduleVideoFrame(frame, _totalFramesScheduled*_timeValue, _timeValue*count, _timeScale );
		_totalFramesScheduled += count;
	}

	HRESULT	STDMETHODCALLTYPE OutputHandler::ScheduledFrameCompleted(IDeckLinkVideoFrame* completedFrame, BMDOutputFrameCompletionResult result)
	{
		// if( completedFrame != _blankFrame ) {
		// 	//LOG(INFO) << "Completed frame != _blankFrame";
		// }

		_buffer->getReadLock();
		if( _buffer->buffer->len > 0 ) {
			//LOG(INFO) << "Scheduling frame with " << int(_buffer->buffer->len) << " bytes of BM SDI Commands";
			scheduleFrame( makeFrameWithSDIProtocol( _deckLinkOutput, _buffer->buffer, true ) );
			//scheduleFrame( addSDIProtocolToFrame( _deckLinkOutput, _blankFrame, _buffer->buffer ) );

			//TODO: How to share this between two outputs
			bmResetBuffer( _buffer->buffer );
		} else {
			// Otherwise schedule a blank frame
			scheduleFrame( _blankFrame  );
		}
		_buffer->releaseReadLock();

		// Can I release the completeFrame?

		return S_OK;
	}

bool OutputHandler::startStreams()
{
	if( !_deckLinkInp)

		LOG(INFO) << "Starting DeckLink output";

		HRESULT result = _deckLinkOutput->StartScheduledPlayback(0, _outputTimeScale, 1.0);
		if(result != S_OK)
		{
			LOG(WARNING) << "Could not start video output - result = " << std::hex << result;
			return false;
}

		return true;
}

bool OutputHandler::stopStreams()
{
		LOG(INFO) << "Stopping DeckLinkOutput streams";
		// // And stop after one frame
		BMDTimeValue actualStopTime;
		HRESULT result = deckLinkOutput()->StopScheduledPlayback(0, &actualStopTime, _outputTimeScale);
		if(result != S_OK)
		{
			LOG(WARNING) << "Could not stop video playback - result = " << std::hex << result;
		}

		LOG(INFO) << "     ...done";
return true;
}


}
