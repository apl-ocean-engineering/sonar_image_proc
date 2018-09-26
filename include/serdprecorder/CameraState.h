#pragma once

#include <memory>

#include "libbmsdi/bmsdi_message.h"
#include "libbmsdi/values.h"


namespace serdprecorder {

template <typename RefStruct, const RefStruct RefTable[] >
class RefTableWrapper {
public:

  RefTableWrapper( int idx = 0 )
    : _tableSize( sizeof(RefTable)/sizeof(RefStruct)),
      _idx(idx) {;}


  typename RefStruct::OrdType ord() { return RefTable[_idx].ord; }
  typename RefStruct::ValType val() { return RefTable[_idx].val; }
  const char *str()                 { return RefTable[_idx].str; }

  const RefStruct entry() { return RefTable[_idx]; }

  bool findOrd( typename RefStruct::OrdType o )
  {
    for( int i = 0; i < _tableSize; ++i ) {
      if( RefTable[i].ord == o ) {
        _idx = i;
        return true;
      }
    }
    return false;
  }

  bool findOrd( typename RefStruct::ValType v )
  {
    for( int i = 0; i < _tableSize; ++i ) {
      if( RefTable[i].val == v ) {
        _idx = i;
        return true;
      }
    }
    return false;
  }

  void operator++() {
    if( _idx < (_tableSize - 1) ) _idx++;
  }

  void operator--() {
    if( _idx > 0 ) _idx--;
  }


  protected:

    size_t _tableSize;
    int _idx;

};

class CameraState {
public:

  CameraState( const std::shared_ptr<SharedBMSDIBuffer> &buffer, int camNum = 1 )
    : _camNum( camNum ),
      _buffer( buffer ),
      _aperture( )
    {;}

    void updateCamera() {
      if( _buffer ) {
        SharedBMSDIBuffer::lock_guard lock( _buffer->writeMutex() );

				bmAddOrdinalAperture( _buffer->buffer, _camNum, _aperture.ord() );
				// bmAddSensorGain( buffer, CamNum, 8 );
				// bmAddReferenceSource( buffer, CamNum, BM_REF_SOURCE_PROGRAM );
				// bmAddAutoWhiteBalance( buffer, CamNum );
      }
    }

    void sendAperture() {
      SDIBufferGuard guard( _buffer );
      guard( [this]( BMSDIBuffer *buffer ){	bmAddOrdinalAperture( buffer, _camNum, _aperture.ord() ); });
    }

    const BmApertureRef apertureInc() {
      ++_aperture;
      sendAperture();
      return _aperture.entry();
    }

    const BmApertureRef apertureDec() {
      ++_aperture;
      sendAperture();
      return _aperture.entry();
    }

protected:


  int _camNum;
  std::shared_ptr<SharedBMSDIBuffer> _buffer;

  RefTableWrapper< struct BmApertureRef, BmApertureTable > _aperture;

};


}
