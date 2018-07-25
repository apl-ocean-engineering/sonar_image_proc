#pragma once

#include <DeckLinkAPI.h>
#include <active_object/active.h>

#include "SDICameraControl.h"

namespace libblackmagic {

	class SharedBMSDIBuffer {
	public:

		typedef std::lock_guard<std::mutex> lock_guard;

		SharedBMSDIBuffer()
			: buffer( bmNewBuffer() ),
				_writeMutex(),
				_readLockMutex(),
				_readLockCount(0)
		{;}

		std::mutex &writeMutex()
			{ return _writeMutex; }

		void getReadLock()
			{
				std::lock_guard<std::mutex> guard(_readLockMutex);
				_writeMutex.try_lock();
				_readLockCount++;
			}

		void releaseReadLock()
			{
				std::lock_guard<std::mutex> guard(_readLockMutex);
				if( --_readLockCount==0 ) _writeMutex.unlock();
			}


		BMSDIBuffer *buffer;

		std::mutex _writeMutex;

		std::mutex _readLockMutex;
		uint8_t _readLockCount;
	};




	class OutputHandler: public IDeckLinkVideoOutputCallback
	{
	public:
		OutputHandler( IDeckLinkOutput *deckLinkOutput,
									 IDeckLinkDisplayMode *mode );
		virtual ~OutputHandler(void);

		//void setBMSDIBuffer( const std::shared_ptr<SharedBMBuffer> &buffer );

		const std::shared_ptr<SharedBMSDIBuffer> &sdiProtocolBuffer();

		HRESULT	STDMETHODCALLTYPE ScheduledFrameCompleted(IDeckLinkVideoFrame* completedFrame, BMDOutputFrameCompletionResult result);

		HRESULT	STDMETHODCALLTYPE ScheduledPlaybackHasStopped(void) {	return S_OK; }

		// Dummy implementations
		HRESULT	STDMETHODCALLTYPE QueryInterface (REFIID iid, LPVOID *ppv){ return E_NOINTERFACE; }
		ULONG STDMETHODCALLTYPE AddRef() { return 1; }
		ULONG STDMETHODCALLTYPE Release() { return 1; }

	protected:

		void scheduleFrame( IDeckLinkVideoFrame *frame, uint8_t count = 1 );

	private:

		IDeckLinkOutput *_deckLinkOutput;
		IDeckLinkDisplayMode *_mode;

		// Cached values
		BMDTimeValue _timeValue;
		BMDTimeScale _timeScale;

		//BMSDIBuffer *_bmsdiBuffer;

		unsigned int _totalFramesScheduled;

		std::shared_ptr<SharedBMSDIBuffer> _buffer;
		IDeckLinkMutableVideoFrame *_blankFrame;

	};

}
