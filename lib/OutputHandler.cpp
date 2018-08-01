
#include <DeckLinkAPI.h>

#include <g3log/g3log.hpp>

#include "SDICameraControl.h"
#include "OutputHandler.h"

namespace libblackmagic {


	OutputHandler::OutputHandler( IDeckLink *deckLink )
			:  _config( bmdModeHD1080p2997 ),								// Set a default
				_enabled(false),
				_deckLink(deckLink),
				_deckLinkOutput( nullptr ),
				_totalFramesScheduled(0),
				_buffer( new SharedBMSDIBuffer() ),
				_blankFrame( nullptr )
		{
		}

	OutputHandler::~OutputHandler(void)
	{
		if( _deckLinkOutput ) _deckLinkOutput->Release();
	}


	IDeckLinkOutput *OutputHandler::deckLinkOutput()
	{
		if( _deckLinkOutput ) return _deckLinkOutput;

		CHECK( S_OK == _deckLink->QueryInterface(IID_IDeckLinkOutput, (void**)&_deckLinkOutput) )
									<< "Could not obtain the IDeckLinkInput interface - result = %08x";

		return _deckLinkOutput;
	}

bool OutputHandler::enable()
{

	    BMDVideoOutputFlags outputFlags  = bmdVideoOutputVANC;
	    HRESULT result;

	    BMDDisplayModeSupport support;
			IDeckLinkDisplayMode *displayMode = nullptr;

	    if( deckLinkOutput()->DoesSupportVideoMode( _config.mode(), 0, outputFlags, &support, &displayMode ) != S_OK) {
	      LOG(WARNING) << "Unable to find a query output modes";
	      return false;
	    }

	    if( support == bmdDisplayModeNotSupported ) {
	      LOG(WARNING) << "Display mode not supported";
	      return false;
	    }
	    // Enable video output
	    if( S_OK != deckLinkOutput()->EnableVideoOutput(_config.mode(), outputFlags ) ) {
	      LOGF(WARNING, "Could not enable video output");
	      return false;
	    }

	    if( S_OK != displayMode->GetFrameRate( &_timeValue, &_timeScale ) ) {
	      LOG(WARNING) << "Unable to get time rate information for output...";
	      return false;
	    }

			//LOG(INFO) << "Time value " << _timeValue << " ; " << _timeScale;

	    // Set the callback object to the DeckLink device's output interface
	    //_outputHandler = new OutputHandler( _deckLinkOutput, displayMode );
	    result = _deckLinkOutput->SetScheduledFrameCompletionCallback( this );
	    if(result != S_OK) {
	      LOGF(WARNING, "Could not set callback - result = %08x\n", result);
	      return false;
	    }

			_config.setMode( displayMode->GetDisplayMode() );
	    displayMode->Release();

			scheduleFrame( blankFrame() );

	    LOG(INFO) << "DeckLinkOutput complete!";
			_enabled = true;
	    return true;
	  }

	void OutputHandler::scheduleFrame( IDeckLinkVideoFrame *frame, uint8_t count )
	{
		LOG(DEBUG) << "Scheduling frame " << _totalFramesScheduled;
		_deckLinkOutput->ScheduleVideoFrame(frame, _totalFramesScheduled*_timeValue, _timeValue*count, _timeScale );
		_totalFramesScheduled += count;
	}

	HRESULT	STDMETHODCALLTYPE OutputHandler::ScheduledFrameCompleted(IDeckLinkVideoFrame* completedFrame, BMDOutputFrameCompletionResult result)
	{
		// LOG(INFO) << "Scheduling another frame";

		if( completedFrame != _blankFrame ) {
			LOG(INFO) << "Completed frame != _blankFrame";
		}

		_buffer->getReadLock();
		if( _buffer->buffer->len > 0 ) {
			LOG(INFO) << "Scheduling frame with " << int(_buffer->buffer->len) << " bytes of BM SDI Commands";
			scheduleFrame( makeFrameWithSDIProtocol( _deckLinkOutput, _buffer->buffer, true ) );
			//scheduleFrame( addSDIProtocolToFrame( _deckLinkOutput, _blankFrame, _buffer->buffer ) );

			//TODO: How to share this between two outputs
			bmResetBuffer( _buffer->buffer );
		} else {
			// Otherwise schedule a blank frame
			scheduleFrame( blankFrame() );
		}
		_buffer->releaseReadLock();

		// Can I release the completeFrame?

		return S_OK;
	}

	bool OutputHandler::startStreams()
	{
		if( !_enabled && !enable() ) return false;

		LOG(INFO) << "Starting DeckLink output ...";

		// // Pre-roll a few blank frames
		// const int prerollFrames = 3;
		// for( int i = 0; i < prerollFrames ; ++i ) {
		// 	scheduleFrame(blankFrame());
		// }

		HRESULT result = _deckLinkOutput->StartScheduledPlayback(0, _timeScale, 1.0);
		if(result != S_OK) {
			LOG(WARNING) << "Could not start video output - result = " << std::hex << result;
			return false;
		}

		LOG(INFO) << "     ... done";
		return true;
}

bool OutputHandler::stopStreams()
{
		LOG(INFO) << "Stopping DeckLinkOutput streams";
		// // And stop after one frame
		BMDTimeValue actualStopTime;
		HRESULT result = deckLinkOutput()->StopScheduledPlayback(0, &actualStopTime, _timeScale);
		if(result != S_OK)
		{
			LOG(WARNING) << "Could not stop video playback - result = " << std::hex << result;
		}

		LOG(INFO) << "     ...done";
return true;
}


}
