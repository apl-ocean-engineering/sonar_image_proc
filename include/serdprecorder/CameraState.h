#pragma once

#include <memory>
#include <iostream>

#include "libbmsdi/bmsdi_message.h"
#include "libbmsdi/helpers.h"
#include "libbmsdi/values.h"


namespace serdprecorder {

  using std::cout;
  using std::endl;

class Bounded {
public:
  Bounded( int idx = 0, int max = 40 )
    : _idx(idx), _min(0), _max(max)
    {;}

    int increment() {
      if( _idx < (_max-1)) ++_idx;
      return _idx;
    }

    int decrement() {
      if( _idx > 0) --_idx;
      return _idx;
    }

    int index() const { return _idx; }

  protected:

    int _idx, _min, _max;
};

template <typename RefStruct, const RefStruct RefTable[], size_t _tableSize >
class RefTableWrapper {
public:

  RefTableWrapper( int idx = 0 )
    :  _idx(idx)
      {cout << "Table size " << _tableSize << endl;}

  typename RefStruct::OrdType ord() { return entry().ord; }
  typename RefStruct::ValType val() { return entry().val; }
  const char *str()                 { return entry().str; }

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

  void increment() {
    cout << "_idx was " << _idx;
    if( _idx < (_tableSize - 1) ) _idx++;
    cout << "_idx is " << _idx << endl;;
  }

  void decrement() {
    cout << "_idx was " << _idx;
    if( _idx > 0 ) _idx--;
    cout << "_idx is " << _idx << endl;
  }


  protected:
    int _idx;

};

using libblackmagic::SharedBMSDIBuffer;
using libblackmagic::SDIBufferGuard;


class CameraState {
public:

  CameraState( const std::shared_ptr<SharedBMSDIBuffer> &buffer, int camNum = 1 )
    : _camNum( camNum ),
      _buffer( buffer ),
      _aperture(0),
      _shutter(10),
      _sensorGain(2)
    {;}

    void updateCamera() {
      if( _buffer ) {
        SharedBMSDIBuffer::lock_guard lock( _buffer->writeMutex() );

				bmAddOrdinalAperture( _buffer->buffer, _camNum, _aperture.index() );
        bmAddOrdinalShutter( _buffer->buffer, _camNum, _shutter.index() );
				bmAddSensorGain( _buffer->buffer, _camNum, _sensorGain.ord() );

        bmAddAutoExposureMode( _buffer->buffer, _camNum, 0 );
				// bmAddReferenceSource( buffer, CamNum, BM_REF_SOURCE_PROGRAM );
				// bmAddAutoWhiteBalance( buffer, CamNum );
      }
    }

    void sendAperture() {
      SDIBufferGuard guard( _buffer );
      guard( [this]( BMSDIBuffer *buffer ){	bmAddOrdinalAperture( buffer, _camNum, _aperture.index() ); });
    }

    const int apertureInc() {
      _aperture.increment();
      sendAperture();
      return _aperture.index();
    }

    const int apertureDec() {
      _aperture.decrement();
      sendAperture();
      return _aperture.index();
    }


    void sendExposure() {
      SDIBufferGuard guard( _buffer );
      guard( [this]( BMSDIBuffer *buffer ){	bmAddOrdinalShutter( buffer, _camNum, _shutter.index() ); });
    }

    const int exposureInc() {
      _shutter.increment();
      sendExposure();
      return _shutter.index();
    }

    const int exposureDec() {
      _shutter.decrement();
      sendExposure();
      return _shutter.index();
    }

    void sendSensorGain() {
      SDIBufferGuard guard( _buffer );
      guard( [this]( BMSDIBuffer *buffer ){	bmAddSensorGain( buffer, _camNum, _sensorGain.ord() ); });
    }

    const BmSensorGainRef gainInc() {
      _sensorGain.increment();
      sendSensorGain();
      return _sensorGain.entry();
    }

    const BmSensorGainRef gainDec() {
      _sensorGain.decrement();
      sendSensorGain();
      return _sensorGain.entry();
    }

protected:


  int _camNum;
  std::shared_ptr<SharedBMSDIBuffer> _buffer;

  //RefTableWrapper< struct BmApertureRef, BmApertureTable > _aperture;
  Bounded _aperture, _shutter;

  RefTableWrapper< struct BmSensorGainRef, BmSensorGainTable, sizeof(BmSensorGainTable)/sizeof(struct BmSensorGainRef) > _sensorGain;

};


}
