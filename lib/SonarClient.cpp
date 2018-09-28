

#include "serdprecorder/SonarClient.h"

namespace serdprecorder {


  SonarClient::SonarClient( const std::string &ipAddr, const shared_ptr<VideoRecorder> &recorder )
  : _ipAddr( ipAddr ),
  _thread(),
  _recorder( recorder ),
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
          if( statusRx.status().wait_for( std::chrono::milliseconds(100) ) ) {

            if( statusRx.status().valid() ) {
              auto addr( statusRx.status().ipAddr() );

              LOG(INFO) << "Using detected sonar at IP address " << addr;

              // statusRx.status().dump();

              dataRx.reset( new DataRx( ioSrv.service(), addr ) );

            }

          }

        } else {

          shared_ptr<SimplePingResult> ping;

          dataRx->queue().wait_for_pop( ping, std::chrono::milliseconds(100) );

          // Do something
          auto valid = ping->validate();
          LOG(INFO) << "Got " << (valid ? "valid" : "invalid") << " ping";

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
