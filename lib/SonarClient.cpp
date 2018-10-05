

#include "serdprecorder/SonarClient.h"

#include "serdprecorder/drawSonar.h"

namespace serdprecorder {


  SonarClient::SonarClient( const std::string &ipAddr,
                            const shared_ptr<VideoRecorder> &recorder,
                            const shared_ptr<OpenCVDisplay> &display )
  : _ipAddr( ipAddr ),
  _thread(),
  _recorder( recorder ),
  _display( display ),
  _done( false )
  {;}


  SonarClient::~SonarClient()
  {
    stop();
  }

  void SonarClient::start() {
    _thread = std::thread( [=] { run(); } );
  }

  void SonarClient::stop() {
    _done = true;
    if( _thread.joinable() ) {
      _thread.join();
    }
  }


  // Runs in thread
  void SonarClient::run() {
    try {
      IoServiceThread ioSrv;

      StatusRx statusRx( ioSrv.service() );
      std::unique_ptr<DataRx> dataRx( nullptr );

      if( _ipAddr != "auto" ) {
        LOG(INFO) << "Connecting to sonar with IP address " << _ipAddr;
        auto addr( boost::asio::ip::address_v4::from_string( _ipAddr ) );

        LOG_IF(FATAL,addr.is_unspecified()) << "Couldn't parse IP address" << _ipAddr;

        dataRx.reset( new DataRx( ioSrv.service(), addr ) );
      }

      ioSrv.start();

      while( !_done ) {

        if( !dataRx ) {

          // Attempt auto detection
          if( statusRx.status().wait_for(std::chrono::seconds(1)) ) {
            if( statusRx.status().valid() ) {
              auto addr( statusRx.status().ipAddr() );
              LOG(INFO) << "Using detected sonar at IP address " << addr;
              dataRx.reset( new DataRx( ioSrv.service(), addr ) );
            }
          }

        } else {

          shared_ptr<SimplePingResult> ping(nullptr);

          if( dataRx->queue().wait_for_pop( ping, std::chrono::milliseconds(100) ) ) {
            // Do something
            auto valid = ping->validate();
            LOG(DEBUG) << "Got " << (valid ? "valid" : "invalid") << " ping";

            // Send to recorder
            _recorder->addSonar( ping->data(), ping->dataSize() );

            if( _display ) _display->showSonar( ping );
          }
        }

      }

      ioSrv.stop();

    }
    catch (std::exception& e)
    {
      LOG(WARNING) << "Exception: " << e.what();
    }


  }

}
