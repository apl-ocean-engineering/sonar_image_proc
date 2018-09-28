#pragma once

#include <memory>

#include <boost/asio.hpp>
#include <boost/bind.hpp>


class IoServiceThread {
public:
    IoServiceThread()
      : _service(),
        _thread() {}

    ~IoServiceThread() {}

    void start()
    {
        if (_thread) return; // running

        _thread.reset(new std::thread(
            boost::bind(&boost::asio::io_service::run, &_service)
        ));
    }

    void stop()
    {
        if (!_thread) return; // stopped

        _service.stop();
        _thread->join();
        _service.reset();
        _thread.reset();
    }

    boost::asio::io_service &service()
    { return _service; }

private:
    boost::asio::io_service _service;
    std::unique_ptr<std::thread> _thread;
};
