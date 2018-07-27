#pragma once

//#include <queue>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "active_object/active.h"
#include "active_object/shared_queue.h"
#include <DeckLinkAPI.h>
#include "ThreadSynchronizer.h"

#include "DataTypes.h"

namespace libblackmagic {

  class InputConfig {
  public:
    InputConfig()
      : _do3D(false),
        _mode( bmdModeDetect )
    {;}

    InputConfig &set3D( bool do3D = true )
      { _do3D = do3D;  return *this; }

    InputConfig &setMode( BMDDisplayMode m )
      { _mode = m; return *this; }


    bool           do3D()         { return _do3D; }
    BMDDisplayMode mode()         { return _mode; }

  private:
    bool _do3D;
    BMDDisplayMode _mode;
  };

  class InputHandler : public IDeckLinkInputCallback
  {
  public:
    InputHandler(  IDeckLink *deckLink );

    // Retrieve the current configuration
    InputConfig &config() { return _config; }

    // Attempts to configure the input stream.   If not called explicitly,
    // will be called automatically by startStreams()
    bool enable( void );

    // Lazy initializer
    IDeckLinkInput *deckLinkInput();
    IDeckLinkOutput *deckLinkOutput();

    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, LPVOID *ppv) { return E_NOINTERFACE; }
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE  Release(void);
    virtual HRESULT STDMETHODCALLTYPE VideoInputFormatChanged(BMDVideoInputFormatChangedEvents, IDeckLinkDisplayMode*, BMDDetectedVideoInputFormatFlags);
    virtual HRESULT STDMETHODCALLTYPE VideoInputFrameArrived(IDeckLinkVideoInputFrame*, IDeckLinkAudioInputPacket*);

    active_object::shared_queue< cv::Mat > &queue() { return _queue; }

    //IDeckLinkOutput *deckLinkOutput() { return _deckLinkOutput; }

    bool startStreams();
    bool stopStreams();

  protected:

    bool process( IDeckLinkVideoFrame *frame, bool isRight = false );

    std::thread processInThread( IDeckLinkVideoFrame *frame, bool isRight = false ) {
          return std::thread([=] { process(frame, isRight); });
      }

  private:

    // bool _stop;
    // int32_t _refCount;

    unsigned long _frameCount;

    InputConfig _config;
    bool _enabled;

    IDeckLink *_deckLink;
    IDeckLinkInput *_deckLinkInput;
    IDeckLinkOutput *_deckLinkOutput;   /// N.b. this doesn't need to be the same as the card output,
                                        // It's used to make new frames for conversion.

    // IDeckLinkDisplayMode *_mode;

    //IDeckLinkVideoConversion *_deckLinkConversion;

    active_object::shared_queue< cv::Mat > _queue, rightQueue;
  };

}
