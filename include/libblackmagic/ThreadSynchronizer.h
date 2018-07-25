/*
 * ThreadMutexObject.h
 *
 *  Created on: 11 May 2012
 *      Author: thomas
 */

//TODO: Hm, the abstractions in this file have gotten a little stale, look at it again

#pragma once

#include <mutex>
#include <condition_variable>

//#include <boost/thread.hpp>
//#include <boost/date_time/posix_time/posix_time.hpp>
// #include <boost/thread/condition_variable.hpp>

namespace libvideoio_bm {

// Simplified version which only handles synchronization (no access to
//  stored boolean value)
class ThreadSynchronizer  {
public:
  typedef std::lock_guard<std::mutex> LockGuard;

  ThreadSynchronizer( void )
    : _ready(false)
  {;}

  void lock( void ) { _mutex.lock(); }
  void unlock( void ) { _mutex.unlock(); }
  std::mutex &mutex() { return _mutex; }

  void notify( void )
  {
      {
        std::lock_guard<std::mutex> lk(_mutex);
        _ready = true;
      }
    _cv.notify_all();
  }

  void reset( void )
  {
    std::unique_lock<std::mutex> lk(_mutex);
    _ready = false;
  }

  // The extra while(!_ready) handles other circumstances which might break the
  // wait (signals?)
  void wait( void )
  {
    std::unique_lock<std::mutex> lk(_mutex);
    while(!_ready) {_cv.wait(lk); }
  }

  template< class Rep, class Period >
  bool wait_for( const std::chrono::duration<Rep, Period> &dur )
  {
    {
      std::unique_lock<std::mutex> lk(_mutex);
      bool timeout(false);
      while( !_ready || timeout ) {
        timeout = (_cv.wait_for(lk, dur) == std::cv_status::no_timeout);
      }
      return timeout ? false : true;
    }
  }

private:

  bool _ready;
  std::mutex _mutex;
  std::condition_variable _cv;

};

}
