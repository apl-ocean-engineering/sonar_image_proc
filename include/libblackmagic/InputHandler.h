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
#include "InputConfig.h"

namespace libblackmagic {



  class InputHandler : public IDeckLinkInputCallback
  {
  public:
    InputHandler( IDeckLink *deckLink );

    // Retrieve the current configuration
    InputConfig &config() { return _config; }

    // Attempts to configure the input stream.   If not called explicitly,
    // will be called automatically by startStreams()
    bool enable( void );

    // Lazy initializers
    IDeckLinkInput *deckLinkInput();
    IDeckLinkOutput *deckLinkOutput();

    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, LPVOID *ppv) { return E_NOINTERFACE; }
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE  Release(void);
    virtual HRESULT STDMETHODCALLTYPE VideoInputFormatChanged(BMDVideoInputFormatChangedEvents, IDeckLinkDisplayMode*, BMDDetectedVideoInputFormatFlags);
    virtual HRESULT STDMETHODCALLTYPE VideoInputFrameArrived(IDeckLinkVideoInputFrame*, IDeckLinkAudioInputPacket*);

    active_object::shared_queue< cv::Mat > &queue() { return _queue; }

    bool startStreams();
    bool stopStreams();

    bool grab( void );
    int getRawImage( int i, cv::Mat &mat );


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

    cv::Mat _grabbedImage;

    // IDeckLinkDisplayMode *_mode;

    //IDeckLinkVideoConversion *_deckLinkConversion;

    active_object::shared_queue< cv::Mat > _queue, rightQueue;
  };

}