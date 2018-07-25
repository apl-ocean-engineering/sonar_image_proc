
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

		// void OutputHandler::setBMSDIBuffer( const std::shared_ptr<SharedBMBuffer> &buffer )
		// {
		// 	_buffer = buffer;
		// }

		void OutputHandler::scheduleFrame( IDeckLinkVideoFrame *frame, uint8_t count )
		{
			//LOG(INFO) << "Scheduled frame " << _totalFramesScheduled;
			_deckLinkOutput->ScheduleVideoFrame(_blankFrame, _totalFramesScheduled*_timeValue, _timeValue*count, _timeScale );
			_totalFramesScheduled += count;
		}

		HRESULT	STDMETHODCALLTYPE OutputHandler::ScheduledFrameCompleted(IDeckLinkVideoFrame* completedFrame, BMDOutputFrameCompletionResult result)
		{
			// For simplicity, this will do just one buffer per frame for now
			// BMSDIBuffer *newCmd = nullptr;
			// int bmSDICount = 0;
			// while( _queue.try_and_pop(newCmd) ) {
			// 	//LOG(INFO) << "Got a SDI protocol command to send!";
			//
			// 	if( ! bmAppendBuffer( _bmsdiBuffer, newCmd ) ) {
			// 		// Failure means there's no room in the buffer
			// 		break;
			// 	}
			//
			// 	bmSDICount++;
			// 	free(newCmd);
			// 	newCmd = nullptr;
			// }

			if( _buffer ) {
				// LOG(INFO) << "Scheduling frame with " << bmSDICount << " BM SDI Commands";
				// scheduleFrame( AddSDICameraControlFrame( _deckLinkOutput, _blankFrame, _bmsdiBuffer ) );
				//
				// bmResetBuffer( _bmsdiBuffer );
				//
				// // If the last command is still hanging around, add it to the buffer
				// if( newCmd ) {
				// 	bmAppendBuffer( _bmsdiBuffer, newCmd );
				// 	free(newCmd);
				// }
			} else {
				// When a video frame completes, reschedule is again...
				scheduleFrame( _blankFrame  );
			}

			// Can I release the completeFrame?

			return S_OK;
		}



}
