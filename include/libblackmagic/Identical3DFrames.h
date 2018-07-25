#pragma once

#include <DeckLinkAPI.h>

namespace libblackmagic {

  class Identical3DFrames : public IDeckLinkMutableVideoFrame, public IDeckLinkVideoFrame3DExtensions {
  public:

    Identical3DFrames( IDeckLinkMutableVideoFrame *data );

    //class IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, LPVOID *ppv);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    //class IDeckLinkVideoFrame
    virtual long GetWidth (void) = 0;
    virtual long GetHeight (void) = 0;
    virtual long GetRowBytes (void) = 0;
    virtual BMDPixelFormat GetPixelFormat (void) = 0;
    virtual BMDFrameFlags GetFlags (void) = 0;
    virtual HRESULT GetBytes (/* out */ void **buffer) = 0;

    virtual HRESULT GetTimecode (/* in */ BMDTimecodeFormat format, /* out */ IDeckLinkTimecode **timecode) = 0;
    virtual HRESULT GetAncillaryData (/* out */ IDeckLinkVideoFrameAncillary **ancillary) = 0;

    //class IDeckLinkMutableVideoFrame
    virtual HRESULT SetFlags (/* in */ BMDFrameFlags newFlags) = 0;

    virtual HRESULT SetTimecode (/* in */ BMDTimecodeFormat format, /* in */ IDeckLinkTimecode *timecode) = 0;
    virtual HRESULT SetTimecodeFromComponents (/* in */ BMDTimecodeFormat format, /* in */ uint8_t hours, /* in */ uint8_t minutes, /* in */ uint8_t seconds, /* in */ uint8_t frames, /* in */ BMDTimecodeFlags flags) = 0;
    virtual HRESULT SetAncillaryData (/* in */ IDeckLinkVideoFrameAncillary *ancillary) = 0;
    virtual HRESULT SetTimecodeUserBits (/* in */ BMDTimecodeFormat format, /* in */ BMDTimecodeUserBits userBits) = 0;

    // class IDeckLinkVideoFrame3DExtensions
    virtual BMDVideo3DPackingFormat Get3DPackingFormat (void) = 0;
    virtual HRESULT GetFrameForRightEye (/* out */ IDeckLinkVideoFrame* *rightEyeFrame) = 0;


  protected:

    ~Identical3DFrames();

    int32_t _refCount;

    IDeckLinkMutableVideoFrame *_data;

  };


}
