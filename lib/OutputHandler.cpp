
#include <DeckLinkAPI.h>

#include <g3log/g3log.hpp>

#include "SDICameraControl.h"
#include "OutputHandler.h"

namespace libblackmagic {


	OutputHandler::OutputHandler( IDeckLinkOutput *deckLinkOutput, IDeckLinkDisplayMode *mode )
			: _deckLinkOutput( deckLinkOutput ),
				_mode( mode ),
				_totalFramesScheduled(0),
				_buffer( new SharedBMSDIBuffer() ),
				_blankFrame( CreateBlueFrame(deckLinkOutput, true ))
		{
			_deckLinkOutput->AddRef();
			_mode->AddRef();

			_mode->GetFrameRate( &_timeValue, &_timeScale );

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

	const std::shared_ptr<SharedBMSDIBuffer> &OutputHandler::sdiProtocolBuffer()
	{
		return _buffer;
	}

	void OutputHandler::scheduleFrame( IDeckLinkVideoFrame *frame, uint8_t count )
	{
		//LOG(INFO) << "Scheduled frame " << _totalFramesScheduled;
		_deckLinkOutput->ScheduleVideoFrame(_blankFrame, _totalFramesScheduled*_timeValue, _timeValue*count, _timeScale );
		_totalFramesScheduled += count;
	}

	HRESULT	STDMETHODCALLTYPE OutputHandler::ScheduledFrameCompleted(IDeckLinkVideoFrame* completedFrame, BMDOutputFrameCompletionResult result)
	{
		_buffer->getReadLock();
		if( _buffer->buffer->len > 0 ) {
			LOG(INFO) << "Scheduling frame with " << int(_buffer->buffer->len) << " bytes of BM SDI Commands";
			scheduleFrame( makeFrameWithSDIProtocol( _deckLinkOutput, _blankFrame, _buffer->buffer ) );

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



}
