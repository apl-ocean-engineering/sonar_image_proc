
#include <cstring>

#include "Identical3DFrames.h"

namespace libblackmagic {

    Identical3DFrames::Identical3DFrames( IDeckLinkMutableVideoFrame *data )
    : _data(data)
    {
      _data->AddRef();
    }

    Identical3DFrames::~Identical3DFrames()
    {
      if( _data ) _data->Release();
    }


    //class IUnknown
    HRESULT STDMETHODCALLTYPE Identical3DFrames::QueryInterface(REFIID iid, LPVOID *ppv) {
      if( memcmp((void*)&iid, (void*)&IID_IDeckLinkVideoFrame3DExtensions, 16) == 0 ) {
          *ppv = this;
          return S_OK;
      }

      return E_NOINTERFACE;
    }

    ULONG Identical3DFrames::AddRef(void)
    {
      return __sync_add_and_fetch(&_refCount, 1);
    }

    ULONG Identical3DFrames::Release(void)
    {
      int32_t newRefValue = __sync_sub_and_fetch(&_refCount, 1);
      if (newRefValue == 0)
      {
        delete this;
        return 0;
      }
      return newRefValue;
    }

    //class IDeckLinkVideoFrame
    long Identical3DFrames::GetWidth (void)
    { return _data->GetWidth(); }

    long Identical3DFrames::GetHeight (void)
    { return _data->GetHeight(); }

    long Identical3DFrames::GetRowBytes (void)
    { return _data->GetRowBytes(); }

    BMDPixelFormat Identical3DFrames::GetPixelFormat (void)
    { return _data->GetPixelFormat(); }

    BMDFrameFlags Identical3DFrames::GetFlags (void)
    { return _data->GetFlags(); }

    HRESULT Identical3DFrames::GetBytes (void **buffer)
    { return _data->GetBytes(buffer); }

    HRESULT Identical3DFrames::GetTimecode (/* in */ BMDTimecodeFormat format, /* out */ IDeckLinkTimecode **timecode)
    { return _data->GetTimecode( format, timecode); }

    HRESULT Identical3DFrames::GetAncillaryData (/* out */ IDeckLinkVideoFrameAncillary **ancillary)
    { return _data->GetAncillaryData(ancillary); }

    //class IDeckLinkMutableVideoFrame
    HRESULT Identical3DFrames::SetFlags (/* in */ BMDFrameFlags newFlags)
    { return _data->SetFlags( newFlags); }

    HRESULT Identical3DFrames::SetTimecode (/* in */ BMDTimecodeFormat format, /* in */ IDeckLinkTimecode *timecode)
    { return _data->SetTimecode( format, timecode ); }

    HRESULT Identical3DFrames::SetTimecodeFromComponents (/* in */ BMDTimecodeFormat format, /* in */ uint8_t hours, /* in */ uint8_t minutes, /* in */ uint8_t seconds, /* in */ uint8_t frames, /* in */ BMDTimecodeFlags flags)
    { return _data->SetTimecodeFromComponents( format, hours, minutes, seconds, frames, flags ); }

    HRESULT Identical3DFrames::SetAncillaryData (/* in */ IDeckLinkVideoFrameAncillary *ancillary)
    { return _data->SetAncillaryData( ancillary ); }

    HRESULT Identical3DFrames::SetTimecodeUserBits (/* in */ BMDTimecodeFormat format, /* in */ BMDTimecodeUserBits userBits)
    { return _data->SetTimecodeUserBits( format, userBits ); }

    // class IDeckLinkVideoFrame3DExtensions
    BMDVideo3DPackingFormat Identical3DFrames::Get3DPackingFormat(void)
    {
      return bmdVideo3DPackingRightOnly;
    }

    HRESULT Identical3DFrames::GetFrameForRightEye(/* out */ IDeckLinkVideoFrame* *rightEyeFrame)
    {
      *rightEyeFrame = this;
      return S_OK;
    }


}
