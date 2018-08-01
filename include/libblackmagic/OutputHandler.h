#pragma once

#include <memory>

#include <DeckLinkAPI.h>
#include <active_object/active.h>

#include "SDICameraControl.h"

#include "InputConfig.h"

#include "SDIMessageBuffer.h"

namespace libblackmagic {

	class OutputHandler: public IDeckLinkVideoOutputCallback
	{
	public:
		OutputHandler( IDeckLink *deckLink );
		virtual ~OutputHandler(void);

		// Retrieve the current configuration
    InputConfig &config() { return _config; }

		// Lazy initializer
		IDeckLinkOutput *deckLinkOutput();

		bool enable( void );

		//void setBMSDIBuffer( const std::shared_ptr<SharedBMBuffer> &buffer );

		const std::shared_ptr<SharedBMSDIBuffer> &sdiProtocolBuffer()
			{ return _buffer; }

		HRESULT	STDMETHODCALLTYPE ScheduledFrameCompleted(IDeckLinkVideoFrame* completedFrame, BMDOutputFrameCompletionResult result);
		HRESULT	STDMETHODCALLTYPE ScheduledPlaybackHasStopped(void) {	return S_OK; }

		// Dummy implementations
		HRESULT	STDMETHODCALLTYPE QueryInterface (REFIID iid, LPVOID *ppv){ return E_NOINTERFACE; }
		ULONG STDMETHODCALLTYPE AddRef() { return 1; }
		ULONG STDMETHODCALLTYPE Release() { return 1; }

		bool startStreams( void );
		bool stopStreams( void );

	protected:

		// Lazy initializer
		IDeckLinkMutableVideoFrame *blankFrame()
			{		if( !_blankFrame ) _blankFrame = makeBlueFrame(deckLinkOutput(), true ); return _blankFrame; }

		void scheduleFrame( IDeckLinkVideoFrame *frame, uint8_t count = 1 );

	private:

		InputConfig _config;
		bool _enabled;

		IDeckLink *_deckLink;
		IDeckLinkOutput *_deckLinkOutput;

		// Cached values
		BMDTimeValue _timeValue;
		BMDTimeScale _timeScale;

		//BMSDIBuffer *_bmsdiBuffer;

		unsigned int _totalFramesScheduled;

		std::shared_ptr<SharedBMSDIBuffer> _buffer;
		IDeckLinkMutableVideoFrame *_blankFrame;

	};

}
