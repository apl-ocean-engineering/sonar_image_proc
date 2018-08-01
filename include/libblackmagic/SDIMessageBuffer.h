#pragma once

#include <memory>

#include "libbmsdi/bmsdi_message.h"

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

}
