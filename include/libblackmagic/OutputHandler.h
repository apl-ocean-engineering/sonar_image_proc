#pragma once

#include <memory>

#include <DeckLinkAPI.h>
#include <active_object/active.h>

#include "SDICameraControl.h"

#include "InputConfig.h"

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


	class SDIBufferGuard {
	public:
			SDIBufferGuard( const std::shared_ptr<SharedBMSDIBuffer> &buffer )
				: _buffer(buffer) {}

				//void operator()( void (*f)( BMSDIBuffer * ) ) {


			template<typename Func>
			void operator()( Func f ) {
				SharedBMSDIBuffer::lock_guard lock( _buffer->writeMutex() );
				f( _buffer->buffer );
			}

			std::shared_ptr<SharedBMSDIBuffer> _buffer;
	};



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
